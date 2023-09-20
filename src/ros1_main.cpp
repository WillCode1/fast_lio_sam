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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "utility/Header.h"
#include "utility/Parameters.h"
#include "system/System.hpp"


int lidar_type;
System slam;
std::string map_frame;
std::string body_frame;
FILE *imu_quat_eular = fopen(DEBUG_FILE_DIR("imu_quat_eular.txt").c_str(), "w");

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
        slam.lidar->oust64_handler(pl_orig_oust, scan);
        break;

    case VELO16:
        pcl::fromROSMsg(*msg, pl_orig_velo);
        slam.lidar->velodyne_handler(pl_orig_velo, scan);
        break;

    default:
        printf("Error LiDAR Type");
        break;
    }

    slam.cache_pointcloud_data(msg->header.stamp.toSec(), scan);
    slam.loger.preprocess_time = timer.elapsedStart();
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
    slam.lidar->avia_handler(pl_orig, scan);
    slam.cache_pointcloud_data(msg->header.stamp.toSec(), scan);
    slam.loger.preprocess_time = timer.elapsedStart();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg)
{
    slam.cache_imu_data(msg->header.stamp.toSec(),
                        V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), 
                        V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
#ifdef Optimize_Use_Imu_Orientation
    slam.imu->imu_orientation = QD(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
#endif
    if (slam.loger.runtime_log)
    {
        static bool flag = false;
        static double start_time = 0;
        if (!flag)
        {
            flag = true;
            start_time = msg->header.stamp.toSec();
        }
        auto rpy = EigenMath::Quaternion2RPY(QD(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
        fprintf(imu_quat_eular, "%0.4lf %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", msg->header.stamp.toSec() - start_time, 0., 0., 0., rpy.x(), rpy.y(), rpy.z());
    }
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
    PointCloudType::Ptr laserCloudWorld(new PointCloudType);
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

void publish_tf(const geometry_msgs::Pose &pose, const double& lidar_end_time)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    q.setW(pose.orientation.w);
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(lidar_end_time), map_frame, body_frame));
}

// 发布里程计
void publish_odometry(const ros::Publisher &pubOdomAftMapped, const state_ikfom &state, const double& lidar_end_time)
{
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose, state);
    pubOdomAftMapped.publish(odomAftMapped);
    publish_tf(odomAftMapped.pose.pose, lidar_end_time);
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
                                        const pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d,
                                        const state_ikfom &state)
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
        auto submap_visual = slam.get_submap_visual(1000, 3, 0.2);
        if (submap_visual == nullptr)
            continue;
        publish_cloud(pubGlobalmap, submap_visual, slam.lidar_end_time, map_frame);
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
    if (slam.map_update_mode)
        slam.relocalization->set_init_pose(init_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SLAM");
    ros::NodeHandle nh;
    bool map_update_mode = false;
    bool save_globalmap_en = false, path_en = true;
    bool scan_pub_en = false, dense_pub_en = false;
    string lidar_topic, imu_topic, config_file;

    ros::param::param("map_update_mode", map_update_mode, false);
    ros::param::param("config_file", config_file, std::string(""));

    load_ros_parameters(string(ROOT_DIR) + config_file, path_en, scan_pub_en, dense_pub_en, lidar_topic, imu_topic, map_frame, body_frame);
    load_parameters(slam, string(ROOT_DIR) + config_file, map_update_mode, save_globalmap_en, lidar_type);

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = lidar_type == AVIA ? nh.subscribe(lidar_topic, 200000, livox_pcl_cbk) : nh.subscribe(lidar_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    // not used
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    // not used
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubImuPath = nh.advertise<nav_msgs::Path>("/imu_path", 100000);
    ros::Publisher pubLidarPath = nh.advertise<nav_msgs::Path>("/lidar_keyframe_trajectory", 100000);

    ros::Publisher pubGlobalmap;
    ros::Publisher pubLoopConstraintEdge;
    std::thread visualizeMapThread;
    if (!slam.map_update_mode)
    {
        pubGlobalmap = nh.advertise<sensor_msgs::PointCloud2>("/map_global", 1);
        visualizeMapThread = std::thread(&visualize_globalmap_thread, pubGlobalmap);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);
    }
    ros::Subscriber sub_initpose = nh.subscribe("/initialpose", 1, initialPoseCallback);
    // ros::Publisher pubground_points = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 100000);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok())
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        if (!slam.sync_sensor_data())
            continue;

        if (slam.run())
        {
            const auto &state = slam.frontend->state;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, state, slam.lidar_end_time);

            /******* Publish points *******/
            if (path_en)
            {
                publish_imu_path(pubImuPath, state, slam.lidar_end_time);
                publish_lidar_keyframe_trajectory(pubLidarPath, *slam.keyframe_pose6d_optimized, slam.lidar_end_time);
            }
            if (scan_pub_en)
                if (dense_pub_en)
                    publish_cloud_world(pubLaserCloudFull, slam.feats_undistort, state, slam.lidar_end_time);
                else
                    publish_cloud_world(pubLaserCloudFull, slam.frontend->feats_down_lidar, state, slam.lidar_end_time);
            // publish_cloud_world(pubground_points, slam.frontend->ground_filter.ground_points, state, slam.lidar_end_time);

            visualize_loop_closure_constraints(pubLoopConstraintEdge, slam.lidar_end_time, slam.loopClosure->loop_constraint_records, slam.loopClosure->copy_keyframe_pose6d, slam.frontend->state);
            // publish_cloud_world(pubLaserCloudEffect, laserCloudOri, state, slam.lidar_end_time);
            if (0)
            {
                PointCloudType::Ptr featsFromMap(new PointCloudType());
                slam.frontend->get_ikdtree_point(featsFromMap);
                publish_ikdtree_map(pubLaserCloudMap, featsFromMap, slam.lidar_end_time);
            }
        }

        rate.sleep();
    }

    slam.save_trajectory();

    if (save_globalmap_en)
        slam.save_globalmap();

    return 0;
}
