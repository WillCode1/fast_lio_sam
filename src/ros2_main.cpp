#include <csignal>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "backend/Header.h"
#include "ParametersRos2.h"
#include "backend/backend/Backend.hpp"
#include "backend/utility/evo_tool.h"
#define EVO

bool showOptimizedPose = true;
double globalMapVisualizationSearchRadius = 1000;
double globalMapVisualizationPoseDensity = 10;
double globalMapVisualizationLeafSize = 1;
int lidar_type;
FastlioOdometry frontend;
Backend backend;
std::string map_frame;
std::string body_frame;
std::string lidar_frame;
FILE *location_log = nullptr;
FILE *imu_quat_eular = fopen(DEBUG_FILE_DIR("imu_quat_eular.txt").c_str(), "w");

bool flg_exit = false;
void SigHandle(int sig)
{
    flg_exit = true;
    LOG_WARN("catch sig %d", sig);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    Timer timer;
    pcl::PointCloud<ouster_ros::Point> pl_orig_oust;
    pcl::PointCloud<velodyne_ros::Point> pl_orig_velo;
    PointCloudType::Ptr scan(new PointCloudType());

    switch (lidar_type)
    {
    case OUST64:
        pcl::fromROSMsg(*msg, pl_orig_oust);
        frontend.lidar->oust64_handler(pl_orig_oust, scan);
        break;

    case VELO16:
        pcl::fromROSMsg(*msg, pl_orig_velo);
        frontend.lidar->velodyne_handler(pl_orig_velo, scan);
        break;

    default:
        printf("Error LiDAR Type");
        break;
    }

    frontend.cache_pointcloud_data(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, scan);
    frontend.loger.preprocess_time = timer.elapsedStart();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    Timer timer;
    auto plsize = msg->point_num;
    PointCloudType::Ptr scan(new PointCloudType());
    PointCloudType::Ptr pl_orig(new PointCloudType());
    pl_orig->reserve(plsize);
    PointType point;
    for (uint i = 1; i < plsize; i++)
    {
        if (((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            point.intensity = msg->points[i].reflectivity;
            point.curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

            pl_orig->points.push_back(point);
        }
    }
    frontend.lidar->avia_handler(pl_orig, scan);
    frontend.cache_pointcloud_data(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, scan);
    frontend.loger.preprocess_time = timer.elapsedStart();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    frontend.cache_imu_data(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
                            V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                            QD(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
}

void gnss_cbk(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    backend.gnss->gnss_handler(GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, V3D(msg->latitude, msg->longitude, msg->altitude)));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, V3D(msg->latitude, msg->longitude, msg->altitude));
}

#ifdef UrbanLoco
void UrbanLoco_cbk(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    backend.gnss->UrbanLoco_handler(GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
                                             V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                                             QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                                             V3D(msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35])));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
                                                 V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                                                 QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));
}
#endif

void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubCloud, PointCloudType::Ptr cloud, const double& lidar_end_time, const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    cloud_msg.header.frame_id = frame_id;
    pubCloud->publish(cloud_msg);
}

void publish_cloud_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull, PointCloudType::Ptr laserCloud, const state_ikfom &state, const double& lidar_end_time)
{
    PointCloudType::Ptr laserCloudWorld(new PointCloudType(laserCloud->size(), 1));
    pointcloudLidarToWorld(laserCloud, laserCloudWorld, state);
    publish_cloud(pubLaserCloudFull, laserCloudWorld, lidar_end_time, map_frame);
}

// 发布ikd-tree地图
void publish_ikdtree_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap, PointCloudType::Ptr featsFromMap, const double& lidar_end_time)
{
    publish_cloud(pubLaserCloudMap, featsFromMap, lidar_end_time, map_frame);
}

template <typename T>
void set_posestamp(T &out, const state_ikfom &state)
{
    out.pose.position.x = state.pos(0);
    out.pose.position.y = state.pos(1);
    out.pose.position.z = state.pos(2);
    out.pose.orientation.x = state.rot.coeffs()[0];
    out.pose.orientation.y = state.rot.coeffs()[1];
    out.pose.orientation.z = state.rot.coeffs()[2];
    out.pose.orientation.w = state.rot.coeffs()[3];
}

void publish_tf(tf2_ros::TransformBroadcaster &broadcaster, const state_ikfom &state, const double &lidar_end_time)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    // imu -> map
    transform_stamped.header.frame_id = map_frame;
    transform_stamped.child_frame_id = body_frame;
    transform_stamped.transform.translation.x = state.pos.x();
    transform_stamped.transform.translation.y = state.pos.y();
    transform_stamped.transform.translation.z = state.pos.z();
    transform_stamped.transform.rotation.x = state.rot.x();
    transform_stamped.transform.rotation.y = state.rot.y();
    transform_stamped.transform.rotation.z = state.rot.z();
    transform_stamped.transform.rotation.w = state.rot.w();
    broadcaster.sendTransform(transform_stamped);

    // lidar -> imu
    transform_stamped.header.frame_id = body_frame;
    transform_stamped.child_frame_id = lidar_frame;
    transform_stamped.transform.translation.x = state.offset_T_L_I.x();
    transform_stamped.transform.translation.y = state.offset_T_L_I.y();
    transform_stamped.transform.translation.z = state.offset_T_L_I.z();
    transform_stamped.transform.rotation.x = state.offset_R_L_I.x();
    transform_stamped.transform.rotation.y = state.offset_R_L_I.y();
    transform_stamped.transform.rotation.z = state.offset_R_L_I.z();
    transform_stamped.transform.rotation.w = state.offset_R_L_I.w();
    broadcaster.sendTransform(transform_stamped);
}

// 发布里程计
void publish_odometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped, tf2_ros::TransformBroadcaster &broadcaster,
                      const state_ikfom &state, const double &lidar_end_time, bool need_publish_tf = true)
{
    nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    set_posestamp(odomAftMapped.pose, state);
    pubOdomAftMapped->publish(odomAftMapped);
    if (need_publish_tf)
        publish_tf(broadcaster, state, lidar_end_time);
}

// 每隔10个发布一下轨迹
void publish_imu_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath, const state_ikfom &state, const double& lidar_end_time)
{
    static geometry_msgs::msg::PoseStamped msg_body_pose;
    set_posestamp(msg_body_pose, state);
    msg_body_pose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    msg_body_pose.header.frame_id = map_frame;

    static nav_msgs::msg::Path path;
    path.header.stamp = msg_body_pose.header.stamp;
    path.header.frame_id = map_frame;

    path.poses.push_back(msg_body_pose);
    pubPath->publish(path);
    /*** if path is too large, the rvis will crash ***/
    if (path.poses.size() >= 300)
    {
        path.poses.erase(path.poses.begin());
    }
}

void publish_lidar_keyframe_trajectory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath, const pcl::PointCloud<PointXYZIRPYT> &trajectory, const double &lidar_end_time)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    path.header.frame_id = map_frame;

    geometry_msgs::msg::PoseStamped msg_lidar_pose;
    for (const auto &point : trajectory)
    {
        msg_lidar_pose.pose.position.x = point.x;
        msg_lidar_pose.pose.position.y = point.y;
        msg_lidar_pose.pose.position.z = point.z;
        auto quat = EigenMath::RPY2Quaternion(V3D(point.roll, point.pitch, point.yaw));
        msg_lidar_pose.pose.orientation.x = quat.x();
        msg_lidar_pose.pose.orientation.y = quat.y();
        msg_lidar_pose.pose.orientation.z = quat.z();
        msg_lidar_pose.pose.orientation.w = quat.w();

        msg_lidar_pose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
        msg_lidar_pose.header.frame_id = map_frame;

        path.poses.push_back(msg_lidar_pose);
    }

    pubPath->publish(path);
}

void visualize_loop_closure_constraints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pubLoopConstraintEdge, const double &timestamp,
                                        const unordered_map<int, int> &loop_constraint_records,
                                        const pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d)
{
    if (loop_constraint_records.empty())
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = map_frame;
    markerNode.header.stamp = rclcpp::Time(timestamp * 1e9);
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = map_frame;
    markerEdge.header.stamp = rclcpp::Time(timestamp * 1e9);
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loop_constraint_records.begin(); it != loop_constraint_records.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::msg::Point p;
        p.x = keyframe_pose6d->points[key_cur].x;
        p.y = keyframe_pose6d->points[key_cur].y;
        p.z = keyframe_pose6d->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = keyframe_pose6d->points[key_pre].x;
        p.y = keyframe_pose6d->points[key_pre].y;
        p.z = keyframe_pose6d->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge->publish(markerArray);
}

void visualize_globalmap_thread(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalmap)
{
    while (!flg_exit)
    {
        this_thread::sleep_for(std::chrono::seconds(1));
        auto submap_visual = backend.get_submap_visual(globalMapVisualizationSearchRadius, globalMapVisualizationPoseDensity, globalMapVisualizationLeafSize, showOptimizedPose);
        if (submap_visual == nullptr)
            continue;
        publish_cloud(pubGlobalmap, submap_visual, frontend.lidar_end_time, map_frame);
    }
}

void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const geometry_msgs::msg::Pose &pose = msg->pose.pose;
    const auto &ori = msg->pose.pose.orientation;
    Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
    auto rpy = EigenMath::Quaternion2RPY(quat);
    // prior pose in map(imu pose)
    Pose init_pose;
    init_pose.x = pose.position.x;
    init_pose.y = pose.position.y;
    init_pose.z = pose.position.z;
    init_pose.roll = rpy.x();
    init_pose.pitch = rpy.y();
    init_pose.yaw = rpy.z();
    backend.relocalization->set_init_pose(init_pose);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fast_lio_sam");
    bool pure_localization = false;
    bool save_globalmap_en = false, path_en = true;
    bool scan_pub_en = false, dense_pub_en = false;
    string lidar_topic, imu_topic, gnss_topic;

    bool save_pgm = false;
    double pgm_resolution;
    float min_z, max_z;
    // location_log = fopen(DEBUG_FILE_DIR("location.log").c_str(), "a");

    node->declare_parameter("showOptimizedPose", true);
    node->declare_parameter("globalMapVisualizationSearchRadius", 1000.);
    node->declare_parameter("globalMapVisualizationPoseDensity", 10.);
    node->declare_parameter("globalMapVisualizationLeafSize", 1.);
    node->get_parameter("showOptimizedPose", showOptimizedPose);
    node->get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
    node->get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
    node->get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

    load_ros_parameters(node, path_en, scan_pub_en, dense_pub_en, lidar_topic, imu_topic, gnss_topic, map_frame, body_frame, lidar_frame);
    load_parameters(node, frontend, backend, save_globalmap_en, lidar_type);
    load_pgm_parameters(node, save_pgm, pgm_resolution, min_z, max_z);

#ifdef EVO
    evo_tool et(DEBUG_FILE_DIR("pose_trajectory.txt"));
#endif

    /*** ROS subscribe initialization ***/
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl2;
    if (lidar_type == AVIA)
        sub_pcl1 = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic, 1000, livox_pcl_cbk);
    else
        sub_pcl2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 1000, standard_pcl_cbk);
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1000, imu_cbk);
    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    auto pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
    // not used
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 1000);
    // not used
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 1000);
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/odom_fix", 1000);
    auto pubImuPath = node->create_publisher<nav_msgs::msg::Path>("/imu_path", 1000);
    auto pubLidarPath = node->create_publisher<nav_msgs::msg::Path>("/lidar_keyframe_trajectory", 1000);
    auto pubOdomNotFix = node->create_publisher<nav_msgs::msg::Odometry>("/odom_not_fix", 1000);

    auto pubGlobalmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_global", 1);
    auto pubLoopConstraintEdge = node->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 1);
    std::thread visualizeMapThread = std::thread(&visualize_globalmap_thread, pubGlobalmap);
    auto sub_initpose = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);
    // auto pubground_points = node->create_publisher<sensor_msgs::PointCloud2>("/ground_points", 1000);

    tf2_ros::TransformBroadcaster broadcaster(node);
    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    while (rclcpp::ok())
    {
        if (flg_exit)
            break;
        rclcpp::spin_some(node);

        if (!frontend.sync_sensor_data())
            continue;

        PointCloudType::Ptr feats_undistort(new PointCloudType());
        PointCloudType::Ptr submap_fix(new PointCloudType());
        PointXYZIRPYT this_pose6d;
        if (frontend.run(feats_undistort))
        {
            auto state = frontend.get_state();
            state2pose(this_pose6d, frontend.lidar_end_time, state);
            backend.run(this_pose6d, feats_undistort, submap_fix);
            if (submap_fix->size())
            {
                pose2state(this_pose6d, state);
                frontend.set_pose(state);
                frontend.ikdtree.reconstruct(submap_fix->points);
            }
#ifdef EVO
            et.save_trajectory(state.pos, state.rot, frontend.lidar_end_time);
#endif
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, broadcaster, state, frontend.lidar_end_time);
            // publish_odometry(pubOdomNotFix, broadcaster, frontend.state_not_fix, frontend.lidar_end_time, false);

            /******* Publish points *******/
            if (path_en)
            {
                publish_imu_path(pubImuPath, state, frontend.lidar_end_time);
                publish_lidar_keyframe_trajectory(pubLidarPath, *backend.keyframe_pose6d_optimized, frontend.lidar_end_time);
            }
            if (scan_pub_en)
                if (dense_pub_en)
                    publish_cloud_world(pubLaserCloudFull, feats_undistort, state, frontend.lidar_end_time);
                else
                    publish_cloud_world(pubLaserCloudFull, frontend.feats_down_lidar, state, frontend.lidar_end_time);

            visualize_loop_closure_constraints(pubLoopConstraintEdge, frontend.lidar_end_time, backend.loopClosure->loop_constraint_records, backend.loopClosure->copy_keyframe_pose6d);
            // publish_cloud_world(pubLaserCloudEffect, laserCloudOri, state, frontend.lidar_end_time);
            if (0)
            {
                PointCloudType::Ptr featsFromMap(new PointCloudType());
                frontend.get_ikdtree_point(featsFromMap);
                publish_ikdtree_map(pubLaserCloudMap, featsFromMap, frontend.lidar_end_time);
            }
        }

        rate.sleep();
    }

    backend.save_trajectory();
    backend.save_factor_graph();
    // backend.save_trajectory_to_other_frame(frontend.get_state().offset_R_L_I, frontend.get_state().offset_T_L_I, "imu");
    // backend.save_trajectory_to_other_frame(QD(M3D(backend.gnss->extrinsic_lidar2gnss.topLeftCorner(3, 3))), backend.gnss->extrinsic_lidar2gnss.topLeftCorner(3, 1), "gnss");

    if (save_globalmap_en)
        backend.save_globalmap();

    if (save_pgm)
        backend.save_pgm(pgm_resolution, min_z, max_z);

    return 0;
}
