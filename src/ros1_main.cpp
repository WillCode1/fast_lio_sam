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
#include "utility/Header.h"
#include "system/System.hpp"


int lidar_type;
System slam;
std::string map_frame;
std::string body_frame;

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
void publish_odometry(const ros::Publisher &pubOdomAftMapped, const state_ikfom &state, const esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, const double& lidar_end_time)
{
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose, state);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        // 设置协方差 P里面先是旋转后是位置 这个POSE里面先是位置后是旋转 所以对应的协方差要对调一下
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }
    pubOdomAftMapped.publish(odomAftMapped);

    publish_tf(odomAftMapped.pose.pose, lidar_end_time);
}

// 每隔10个发布一下轨迹
void publish_path(const ros::Publisher &pubPath, const state_ikfom &state, const double& lidar_end_time)
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

void visualize_globalmap_thread(const ros::Publisher &pubGlobalmap)
{
    while (!flg_exit)
    {
        this_thread::sleep_for(std::chrono::seconds(1));
        // if (pubGlobalmap.getNumSubscribers() == 0)
            // continue;
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
    auto rpy = EigenRotation::Quaternion2RPY(quat);
    Pose init_pose;
    init_pose.x = pose.position.x;
    init_pose.y = pose.position.y;
    init_pose.z = pose.position.z;
    init_pose.roll = rpy.x();
    init_pose.pitch = rpy.y();
    init_pose.yaw = rpy.z();
    if (slam.localization_mode)
        slam.relocalization->set_init_pose(init_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SLAM");
    ros::NodeHandle nh;
    double blind, detect_range;
    int n_scans, scan_rate, time_unit;
    vector<double> extrinT;
    vector<double> extrinR;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    bool save_globalmap_en = false, path_en = true;
    string lid_topic, imu_topic;
    bool scan_pub_en = false, dense_pub_en = false;
    bool pure_localization = false;

    nh.param<bool>("localization_mode", pure_localization, false);
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<int>("mapping/max_iteration", slam.frontend->num_max_iterations, 4);
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<string>("common/map_frame", map_frame, "camera_init");
    nh.param<string>("common/body_frame", body_frame, "");
    nh.param<double>("common/timedelay_lidar2imu", slam.timedelay_lidar2imu, 0.0);
    nh.param<double>("mapping/surf_frame_ds_res", slam.frontend->surf_frame_ds_res, 0.5);
    nh.param<int>("mapping/point_skip_num", slam.frontend->point_skip_num, 2);
    nh.param<double>("mapping/ikdtree_resolution", slam.frontend->ikdtree_resolution, 0.5);
    nh.param<double>("mapping/lidar_model_search_range", slam.frontend->lidar_model_search_range, 5);
    nh.param<double>("mapping/cube_side_length", slam.frontend->cube_len, 200);
    nh.param<float>("mapping/keyframe_add_dist_threshold", slam.backend->keyframe_add_dist_threshold, 1);
    nh.param<float>("mapping/keyframe_add_angle_threshold", slam.backend->keyframe_add_angle_threshold, 0.2);
    nh.param<float>("mapping/pose_cov_threshold", slam.backend->pose_cov_threshold, 25);
    nh.param<bool>("mapping/recontruct_kdtree", slam.backend->recontruct_kdtree, true);
    nh.param<float>("mapping/ikdtree_reconstruct_keyframe_num", slam.backend->ikdtree_reconstruct_keyframe_num, 10);
    nh.param<float>("mapping/ikdtree_reconstruct_downsamp_size", slam.backend->ikdtree_reconstruct_downsamp_size, 0.1);
    nh.param<bool>("mapping/loop_closure_enable_flag", slam.loop_closure_enable_flag, false);
    nh.param<bool>("mapping/manually_fine_tune_loop_closure", slam.loopClosure->manually_fine_tune_loop_closure, false);
    nh.param<int>("mapping/loop_keyframe_num_thld", slam.loopClosure->loop_keyframe_num_thld, 50);
    nh.param<float>("mapping/loop_closure_search_radius", slam.loopClosure->loop_closure_search_radius, 10);
    nh.param<float>("mapping/loop_closure_search_time_interval", slam.loopClosure->loop_closure_search_time_interval, 30);
    nh.param<int>("mapping/keyframe_search_num", slam.loopClosure->keyframe_search_num, 20);
    nh.param<float>("mapping/loop_closure_fitness_score_thld", slam.loopClosure->loop_closure_fitness_score_thld, 0.05);
    nh.param<float>("mapping/icp_downsamp_size", slam.loopClosure->icp_downsamp_size, 0.1);
    nh.param<vector<double>>("mapping/loop_closure_vaild_time_period", slam.loopClosure->loop_closure_vaild_time_period, vector<double>());
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", blind, 0.01);
    nh.param<double>("preprocess/det_range", detect_range, 300.f);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", n_scans, 16);
    nh.param<int>("preprocess/timestamp_unit", time_unit, US);
    nh.param<int>("preprocess/scan_rate", scan_rate, 10);
    nh.param<int>("mapping/runtime_log_enable", slam.loger.runtime_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", slam.frontend->extrinsic_est_en, true);
    nh.param<bool>("official/save_globalmap_en", save_globalmap_en, false);
    nh.param<bool>("official/save_keyframe_en", slam.save_keyframe_en, false);
    nh.param<float>("official/save_resolution", slam.save_resolution, 0.1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    cout << "current lidar_type: " << lidar_type << endl;

    BnbOptions match_option;
    Pose init_pose, lidar_pose;
    ros::param::param("bnb3d/algorithm_type", match_option.algorithm_type, std::string("UNKONW"));
    ros::param::param("bnb3d/linear_xy_window_size", match_option.linear_xy_window_size, 10.);
    ros::param::param("bnb3d/linear_z_window_size", match_option.linear_z_window_size, 1.);
    ros::param::param("bnb3d/angular_search_window", match_option.angular_search_window, 30.);
    ros::param::param("bnb3d/pc_resolutions", match_option.pc_resolutions, std::vector<double>());
    ros::param::param("bnb3d/bnb_depth", match_option.bnb_depth, 5);
    ros::param::param("bnb3d/bnb_min_score", match_option.min_score, 0.1);
    ros::param::param("bnb3d/min_xy_resolution", match_option.min_xy_resolution, 0.2);
    ros::param::param("bnb3d/min_z_resolution", match_option.min_z_resolution, 0.1);
    ros::param::param("bnb3d/min_angular_resolution", match_option.min_angular_resolution, 0.1);
    ros::param::param("bnb3d/thread_num", match_option.thread_num, 4);
    ros::param::param("bnb3d/filter_size_scan", match_option.filter_size_scan, 0.1);
    ros::param::param("bnb3d/debug_mode", match_option.debug_mode, false);

    ros::param::param("bnb3d/init_pose/x", init_pose.x, 0.);
    ros::param::param("bnb3d/init_pose/y", init_pose.y, 0.);
    ros::param::param("bnb3d/init_pose/z", init_pose.z, 0.);
    ros::param::param("bnb3d/init_pose/roll", init_pose.roll, 0.);
    ros::param::param("bnb3d/init_pose/pitch", init_pose.pitch, 0.);
    ros::param::param("bnb3d/init_pose/yaw", init_pose.yaw, 0.);

    ros::param::param("bnb3d/lidar_ext/x", lidar_pose.x, 0.);
    ros::param::param("bnb3d/lidar_ext/y", lidar_pose.y, 0.);
    ros::param::param("bnb3d/lidar_ext/z", lidar_pose.z, 0.);
    ros::param::param("bnb3d/lidar_ext/roll", lidar_pose.roll, 0.);
    ros::param::param("bnb3d/lidar_ext/pitch", lidar_pose.pitch, 0.);
    ros::param::param("bnb3d/lidar_ext/yaw", lidar_pose.yaw, 0.);
    slam.relocalization->set_bnb3d_param(match_option, init_pose, lidar_pose);

    int min_plane_point;
    double filter_radius, cluster_dis, plane_dis, plane_point_percent;
    nh.param<double>("gicp/filter_radius", filter_radius, 1);
    nh.param<int>("gicp/min_plane_point", min_plane_point, 20);
    nh.param<double>("gicp/cluster_dis", cluster_dis, 0.1);
    nh.param<double>("gicp/plane_dis", plane_dis, 0.1);
    nh.param<double>("gicp/plane_point_percent", plane_point_percent, 0.1);
    slam.relocalization->set_plane_extract_param(filter_radius, min_plane_point, cluster_dis, plane_dis, plane_point_percent);

    double gicp_downsample, search_radius, teps, feps, fitness_score;
    nh.param<double>("gicp/gicp_downsample", gicp_downsample, 0.2);
    nh.param<double>("gicp/search_radius", search_radius, 0.5);
    nh.param<double>("gicp/teps", teps, 1e-3);
    nh.param<double>("gicp/feps", feps, 1e-3);
    nh.param<double>("gicp/fitness_score", fitness_score, 0.3);
    slam.relocalization->set_gicp_param(gicp_downsample, search_radius, teps, feps, fitness_score);

    vector<double> gravity;
    nh.param<vector<double>>("localization/gravity", gravity, vector<double>());

    slam.lidar->init(n_scans, scan_rate, time_unit, blind, detect_range);
    slam.imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    slam.imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    slam.imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    slam.imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // imu = R * lidar + t
    V3D Lidar_T_wrt_IMU;
    M3D Lidar_R_wrt_IMU;
    V3D gravity_vec;
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    gravity_vec << VEC_FROM_ARRAY(gravity);
    slam.frontend->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, gravity_vec);
    slam.init_system_mode(pure_localization);

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_initpose = nh.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber sub_pcl = lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    // not used
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    // not used
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);

    ros::Publisher pubGlobalmap = nh.advertise<sensor_msgs::PointCloud2>("/map_global", 1);
    std::thread visualizeMapThread(&visualize_globalmap_thread, pubGlobalmap);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok())
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        if (slam.run())
        {
            const auto &state = slam.frontend->state;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, state, slam.frontend->kf, slam.lidar_end_time);

            /******* Publish points *******/
            if (path_en)
                publish_path(pubPath, state, slam.lidar_end_time);
            if (scan_pub_en)
                if (dense_pub_en)
                    publish_cloud_world(pubLaserCloudFull, slam.feats_undistort, state, slam.lidar_end_time);
                else
                    publish_cloud_world(pubLaserCloudFull, slam.frontend->feats_down_lidar, state, slam.lidar_end_time);

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
