#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "backend/Header.h"
#include "ParametersRos1.h"
#include "backend/backend/Backend.hpp"
#include "backend/utility/evo_tool.h"
#include "slam_interfaces/InsPvax.h"
// #define EVO
// #define UrbanLoco
// #define liosam


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
ros::Publisher pubGpsIns;
FILE *location_log = nullptr;

bool flg_exit = false;
void SigHandle(int sig)
{
    flg_exit = true;
    LOG_WARN("catch sig %d", sig);
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
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

    frontend.cache_pointcloud_data(msg->header.stamp.toSec(), scan);
    frontend.loger.preprocess_time = timer.elapsedStart();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
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
    frontend.cache_pointcloud_data(msg->header.stamp.toSec(), scan);
    frontend.loger.preprocess_time = timer.elapsedStart();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg)
{
    frontend.cache_imu_data(msg->header.stamp.toSec(),
                            V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                            QD(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
}

void gnss_cbk(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    V3D gnss_position = backend.gnss->gnss_global2local(V3D(RAD2DEG(msg->latitude), RAD2DEG(msg->longitude), msg->altitude));
    backend.gnss->gnss_handler(GnssPose(msg->header.stamp.toSec(), gnss_position));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.toSec(), gnss_position);
}

void UrbanLoco_cbk(const nav_msgs::OdometryConstPtr &msg)
{
    V3D gnss_position = backend.gnss->gnss_global2local(V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    backend.gnss->gnss_handler(GnssPose(msg->header.stamp.toSec(), gnss_position,
                                        QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
                                        V3D(msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35])));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.toSec(), gnss_position,
                                                 QD(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));
}

void gnss_ins_cbk(const slam_interfaces::InsPvax::ConstPtr &msg)
{
    if (msg->ins_status != 0xFFFFFFFF)
        return;

    V3D gnss_position = backend.gnss->gnss_global2local(V3D(RAD2DEG(msg->latitude), RAD2DEG(msg->longitude), msg->altitude));

#if 0
    geometry_msgs::Twist gps_pose;
    gps_pose.linear.x = gnss_position.x();
    gps_pose.linear.y = gnss_position.y();
    gps_pose.linear.z = gnss_position.z();
    gps_pose.angular.x = RAD2DEG(msg->roll);
    gps_pose.angular.y = RAD2DEG(msg->pitch);
    gps_pose.angular.z = RAD2DEG(msg->azimuth);
    pubGpsIns.publish(gps_pose);
#endif

    if (msg->position_status != 4 || msg->heading_status != 4)
        return;

    if (msg->numsv <= backend.gnss->numsv)
        return;

    if (msg->rtk_age > backend.gnss->rtk_age)
        return;

    QD rot = EigenMath::RPY2Quaternion(V3D(msg->roll, msg->pitch, msg->azimuth));
    Eigen::VectorXd pose_std(6);
    pose_std << msg->latitude_std, msg->longitude_std, msg->altitude_std, msg->roll_std, msg->pitch_std, msg->azimuth_std;
    backend.gnss->gnss_handler(GnssPose(msg->header.stamp.toSec(), gnss_position, rot, pose_std));
    backend.relocalization->gnss_pose = GnssPose(msg->header.stamp.toSec(), gnss_position, rot);
}

void publish_cloud(const ros::Publisher &pubCloud, PointCloudType::Ptr cloud, const double& lidar_end_time, const std::string& frame_id)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
    cloud_msg.header.frame_id = frame_id;
    pubCloud.publish(cloud_msg);
}

void publish_cloud_world(const ros::Publisher &pubLaserCloudFull, PointCloudType::Ptr laserCloud, const state_ikfom &state, const double& lidar_end_time)
{
    PointCloudType::Ptr laserCloudWorld(new PointCloudType(laserCloud->size(), 1));
    pointcloudLidarToWorld(laserCloud, laserCloudWorld, state);
    publish_cloud(pubLaserCloudFull, laserCloudWorld, lidar_end_time, map_frame);
}

// 发布ikd-tree地图
void publish_ikdtree_map(const ros::Publisher &pubLaserCloudMap, PointCloudType::Ptr featsFromMap, const double& lidar_end_time)
{
    publish_cloud(pubLaserCloudMap, featsFromMap, lidar_end_time, map_frame);
}

// 设置输出的t,q，在publish_odometry，publish_path调用
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

void publish_tf(const geometry_msgs::Pose &pose, const state_ikfom &state, const double& lidar_end_time)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // imu -> map
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(lidar_end_time), map_frame, body_frame));

    // lidar -> imu
    transform.setOrigin(tf::Vector3(state.offset_T_L_I.x(), state.offset_T_L_I.y(), state.offset_T_L_I.z()));
    q.setValue(state.offset_R_L_I.x(), state.offset_R_L_I.y(), state.offset_R_L_I.z(), state.offset_R_L_I.w());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(lidar_end_time), body_frame, lidar_frame));
}

// 发布里程计
void publish_odometry(const ros::Publisher &pubOdomAftMapped, const state_ikfom &state, const double& lidar_end_time, bool need_publish_tf = true)
{
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose, state);
    pubOdomAftMapped.publish(odomAftMapped);
    if (need_publish_tf)
        publish_tf(odomAftMapped.pose.pose, state, lidar_end_time);
}

void publish_imu_path(const ros::Publisher &pubPath, const state_ikfom &state, const double& lidar_end_time)
{
    static geometry_msgs::PoseStamped msg_body_pose;
    set_posestamp(msg_body_pose, state);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = map_frame;

    static nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame;

    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
    /*** if path is too large, the rvis will crash ***/
    if (path.poses.size() >= 300)
    {
        path.poses.erase(path.poses.begin());
    }
}

void publish_lidar_keyframe_trajectory(const ros::Publisher &pubPath, const pcl::PointCloud<PointXYZIRPYT> &trajectory, const double &lidar_end_time)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = map_frame;

    geometry_msgs::PoseStamped msg_lidar_pose;
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

        msg_lidar_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
        msg_lidar_pose.header.frame_id = map_frame;

        path.poses.push_back(msg_lidar_pose);
    }

    pubPath.publish(path);
}

void visualize_loop_closure_constraints(const ros::Publisher &pubLoopConstraintEdge, const double &timestamp,
                                        const unordered_map<int, int> &loop_constraint_records,
                                        const pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d)
{
    if (loop_constraint_records.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = map_frame;
    markerNode.header.stamp = ros::Time().fromSec(timestamp);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
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
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = map_frame;
    markerEdge.header.stamp = ros::Time().fromSec(timestamp);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
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
        geometry_msgs::Point p;
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
    pubLoopConstraintEdge.publish(markerArray);
}

void visualize_globalmap_thread(const ros::Publisher &pubGlobalmap)
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

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    const geometry_msgs::Pose &pose = msg->pose.pose;
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
    ros::init(argc, argv, "SLAM");
    ros::NodeHandle nh;
    bool save_globalmap_en = false, path_en = true;
    bool scan_pub_en = false, dense_pub_en = false;
    string lidar_topic, imu_topic, gnss_topic;

    bool save_pgm = false;
    double pgm_resolution;
    float min_z, max_z;
    std::vector<double> lla;
    // location_log = fopen(DEBUG_FILE_DIR("location.log").c_str(), "a");

    ros::param::param("showOptimizedPose", showOptimizedPose, true);
    ros::param::param("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1000.);
    ros::param::param("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.);
    ros::param::param("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.);
    ros::param::param("mapping/lla", lla, std::vector<double>());

    load_ros_parameters(path_en, scan_pub_en, dense_pub_en, lidar_topic, imu_topic, gnss_topic, map_frame, body_frame, lidar_frame);
    load_parameters(frontend, backend, save_globalmap_en, lidar_type);
    load_pgm_parameters(save_pgm, pgm_resolution, min_z, max_z);

    if (lla.size() == 3 && lla[0] != 0)
    {
#ifdef ENU
        enu_coordinate::Earth::SetOrigin(V3D(lla[0], lla[1], lla[2]));
#else
        utm_coordinate::SetUtmOrigin(V3D(lla[0], lla[1], lla[2]));
#endif
    }

#ifdef EVO
    evo_tool et(DEBUG_FILE_DIR("pose_trajectory.txt"));
#endif

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = lidar_type == AVIA ? nh.subscribe(lidar_topic, 200000, livox_pcl_cbk) : nh.subscribe(lidar_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
#ifdef UrbanLoco
    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, UrbanLoco_cbk);
#elif defined(liosam)
    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk);
#else
    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_ins_cbk);
#endif
    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    // not used
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    // not used
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/odom_fix", 100000);
    ros::Publisher pubImuPath = nh.advertise<nav_msgs::Path>("/imu_path", 100000);
    ros::Publisher pubLidarPath = nh.advertise<nav_msgs::Path>("/lidar_keyframe_trajectory", 100000);
    ros::Publisher pubOdomNotFix = nh.advertise<nav_msgs::Odometry>("/odom_not_fix", 100000);

    ros::Publisher pubGlobalmap = nh.advertise<sensor_msgs::PointCloud2>("/map_global", 1);
    ros::Publisher pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);
    std::thread visualizeMapThread = std::thread(&visualize_globalmap_thread, pubGlobalmap);
    ros::Subscriber sub_initpose = nh.subscribe("/initialpose", 1, initialPoseCallback);
    // ros::Publisher pubground_points = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 100000);
    pubGpsIns = nh.advertise<geometry_msgs::Twist>("/gps_ins_pose", 100000);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok())
    {
        if (flg_exit)
            break;
        ros::spinOnce();

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
            publish_odometry(pubOdomAftMapped, state, frontend.lidar_end_time);
            publish_odometry(pubOdomNotFix, frontend.state_not_fix, frontend.lidar_end_time, false);

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
                    publish_cloud(pubLaserCloudFull, frontend.feats_down_world, frontend.lidar_end_time, map_frame);
            // publish_cloud_world(pubground_points, frontend.ground_filter.ground_points, state, frontend.lidar_end_time);

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
