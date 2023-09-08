#include <csignal>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include "livox_interfaces2/msg/custom_msg.hpp"
// #include <livox_ros_driver/CustomMsg.h>
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

    slam.cache_pointcloud_data(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, scan);
    slam.loger.preprocess_time = timer.elapsedStart();
}

void livox_pcl_cbk(const livox_interfaces2::msg::CustomMsg::SharedPtr msg)
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
    slam.cache_pointcloud_data(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9, scan);
    slam.loger.preprocess_time = timer.elapsedStart();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    slam.cache_imu_data(msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9,
                        V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                        V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
}

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
    PointCloudType::Ptr laserCloudWorld(new PointCloudType);
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

// 发布里程计
void publish_odometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped, tf2_ros::TransformBroadcaster &broadcaster,
                      const state_ikfom &state, const esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, const double &lidar_end_time)
{
    static nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    set_posestamp(odomAftMapped.pose, state);
    pubOdomAftMapped->publish(odomAftMapped);
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

    static geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odomAftMapped.header.stamp;
    transform_stamped.header.frame_id = map_frame;
    transform_stamped.child_frame_id = body_frame;
    transform_stamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform_stamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform_stamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform_stamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    broadcaster.sendTransform(transform_stamped);
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


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("SLAM");

    double blind, detect_range;
    int n_scans, scan_rate, time_unit;
    vector<double> extrinT;
    vector<double> extrinR;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    bool save_globalmap_en = false, path_en = true;
    string lid_topic, imu_topic;
    int num_max_iterations = 0;
    bool scan_pub_en = false, dense_pub_en = false;
    bool pure_localization = false;

    node->get_parameter_or("localization_mode", pure_localization, false);
    node->get_parameter_or("publish/path_en", path_en, true);
    node->get_parameter_or("publish/scan_publish_en", scan_pub_en, true);
    node->get_parameter_or("publish/dense_publish_en", dense_pub_en, true);
    node->get_parameter_or("common/lid_topic", lid_topic, std::string("/livox/lidar"));
    node->get_parameter_or("common/imu_topic", imu_topic, std::string("/livox/imu"));
    node->get_parameter_or("common/map", map_frame, std::string("camera_init"));
    node->get_parameter_or("common/body_frame", body_frame, std::string(""));

    node->get_parameter_or("mapping/max_iteration", num_max_iterations, 4);
    node->get_parameter_or("common/timedelay_lidar2imu", slam.timedelay_lidar2imu, 0.0);
    node->get_parameter_or("mapping/surf_frame_ds_res", slam.frontend->surf_frame_ds_res, 0.5);
    node->get_parameter_or("mapping/lidar_model_search_range", slam.frontend->lidar_model_search_range, 5);
    node->get_parameter_or("mapping/point_skip_num", slam.frontend->point_skip_num, 2);
    node->get_parameter_or("mapping/ikdtree_resolution", slam.frontend->ikdtree_resolution, 0.5);
    node->get_parameter_or("mapping/cube_side_length", slam.frontend->cube_len, 200.);
    node->get_parameter_or("mapping/keyframe_add_dist_threshold", slam.backend->keyframe_add_dist_threshold, 1);
    node->get_parameter_or("mapping/keyframe_add_angle_threshold", slam.backend->keyframe_add_angle_threshold, 0.2);
    node->get_parameter_or("mapping/pose_cov_threshold", slam.backend->pose_cov_threshold, 25);
    node->get_parameter_or("mapping/gnssValidInterval", slam.gnss->gnssValidInterval, 0.2);
    node->get_parameter_or("mapping/gpsCovThreshold", slam.gnss->gpsCovThreshold, 2);
    node->get_parameter_or("mapping/useGpsElevation", slam.gnss->useGpsElevation, false);
    node->get_parameter_or("mapping/recontruct_kdtree", slam.backend->recontruct_kdtree, true);
    node->get_parameter_or("mapping/ikdtree_reconstruct_keyframe_num", slam.backend->ikdtree_reconstruct_keyframe_num, 10);
    node->get_parameter_or("mapping/ikdtree_reconstruct_downsamp_size", slam.backend->ikdtree_reconstruct_downsamp_size, 0.1);
    node->get_parameter_or("mapping/loop_closure_enable_flag", slam.loop_closure_enable_flag, false);
    node->get_parameter_or("mapping/manually_fine_tune_loop_closure", slam.loopClosure->manually_fine_tune_loop_closure, false);
    node->get_parameter_or("mapping/loop_keyframe_num_thld", slam.loopClosure->loop_keyframe_num_thld, 50);
    node->get_parameter_or("mapping/loop_closure_search_radius", slam.loopClosure->loop_closure_search_radius, 10);
    node->get_parameter_or("mapping/loop_closure_search_time_interval", slam.loopClosure->loop_closure_search_time_interval, 30);
    node->get_parameter_or("mapping/keyframe_search_num", slam.loopClosure->keyframe_search_num, 20);
    node->get_parameter_or("mapping/loop_closure_fitness_score_thld", slam.loopClosure->loop_closure_fitness_score_thld, 0.05);
    node->get_parameter_or("mapping/icp_downsamp_size", slam.loopClosure->icp_downsamp_size, 0.1);
    node->get_parameter_or("mapping/odom_loop_vaild_period", slam.loopClosure->loop_vaild_period["odom"], vector<double>());
    node->get_parameter_or("mapping/scancontext_loop_vaild_period", slam.loopClosure->loop_vaild_period["scancontext"], vector<double>());
    node->get_parameter_or("mapping/gyr_cov", gyr_cov, 0.1);
    node->get_parameter_or("mapping/acc_cov", acc_cov, 0.1);
    node->get_parameter_or("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    node->get_parameter_or("mapping/b_acc_cov", b_acc_cov, 0.0001);
    node->get_parameter_or("preprocess/blind", blind, 0.01);
    node->get_parameter_or("preprocess/det_range", detect_range, 300.);
    node->get_parameter_or("preprocess/lidar_type", lidar_type, int(AVIA));
    node->get_parameter_or("preprocess/scan_line", n_scans, 16);
    node->get_parameter_or("preprocess/timestamp_unit", time_unit, int(US));
    node->get_parameter_or("preprocess/scan_rate", scan_rate, 10);
    node->get_parameter_or("mapping/runtime_log_enable", slam.loger.runtime_log, 0);
    node->get_parameter_or("mapping/extrinsic_est_en", slam.frontend->extrinsic_est_en, true);
    node->get_parameter_or("official/save_globalmap_en", save_globalmap_en, false);
    node->get_parameter_or("official/save_keyframe_en", slam.save_keyframe_en, false);
    node->get_parameter_or("official/save_resolution", slam.save_resolution, 0.1f);
    node->get_parameter_or("mapping/extrinsic_T", extrinT, vector<double>());
    node->get_parameter_or("mapping/extrinsic_R", extrinR, vector<double>());
    cout << "current lidar_type: " << lidar_type << endl;

    node->get_parameter_or("scan_context/lidar_height", slam.relocalization->sc_manager->LIDAR_HEIGHT, 2.0);
    node->get_parameter_or("scan_context/sc_dist_thres", slam.relocalization->sc_manager->SC_DIST_THRES, 0.5);

    if (pure_localization)
    {
        node->get_parameter_or("relocalization_cfg/algorithm_type", slam.relocalization->algorithm_type, std::string("UNKONW"));

        BnbOptions match_option;
        node->get_parameter_or("bnb3d/linear_xy_window_size", match_option.linear_xy_window_size, 10.);
        node->get_parameter_or("bnb3d/linear_z_window_size", match_option.linear_z_window_size, 1.);
        node->get_parameter_or("bnb3d/angular_search_window", match_option.angular_search_window, 30.);
        node->get_parameter_or("bnb3d/pc_resolutions", match_option.pc_resolutions, std::vector<double>());
        node->get_parameter_or("bnb3d/bnb_depth", match_option.bnb_depth, 5);
        node->get_parameter_or("bnb3d/bnb_min_score", match_option.min_score, 0.1);
        node->get_parameter_or("bnb3d/min_xy_resolution", match_option.min_xy_resolution, 0.2);
        node->get_parameter_or("bnb3d/min_z_resolution", match_option.min_z_resolution, 0.1);
        node->get_parameter_or("bnb3d/min_angular_resolution", match_option.min_angular_resolution, 0.1);
        node->get_parameter_or("bnb3d/thread_num", match_option.thread_num, 4);
        node->get_parameter_or("bnb3d/filter_size_scan", match_option.filter_size_scan, 0.1);
        node->get_parameter_or("bnb3d/debug_mode", match_option.debug_mode, false);

        Pose lidar_extrinsic;
        node->get_parameter_or("relocalization_cfg/lidar_ext/x", lidar_extrinsic.x, 0.);
        node->get_parameter_or("relocalization_cfg/lidar_ext/y", lidar_extrinsic.y, 0.);
        node->get_parameter_or("relocalization_cfg/lidar_ext/z", lidar_extrinsic.z, 0.);
        node->get_parameter_or("relocalization_cfg/lidar_ext/roll", lidar_extrinsic.roll, 0.);
        node->get_parameter_or("relocalization_cfg/lidar_ext/pitch", lidar_extrinsic.pitch, 0.);
        node->get_parameter_or("relocalization_cfg/lidar_ext/yaw", lidar_extrinsic.yaw, 0.);
        slam.relocalization->set_bnb3d_param(match_option, lidar_extrinsic);

        double step_size, resolution;
        node->get_parameter_or("ndt/step_size", step_size, 0.1);
        node->get_parameter_or("ndt/resolution", resolution, 1);
        slam.relocalization->set_ndt_param(step_size, resolution);

        bool use_gicp;
        double gicp_downsample, search_radius, teps, feps, fitness_score;
        node->get_parameter_or("gicp/use_gicp", use_gicp, true);
        node->get_parameter_or("gicp/gicp_downsample", gicp_downsample, 0.2);
        node->get_parameter_or("gicp/search_radius", search_radius, 0.5);
        node->get_parameter_or("gicp/teps", teps, 1e-3);
        node->get_parameter_or("gicp/feps", feps, 1e-3);
        node->get_parameter_or("gicp/fitness_score", fitness_score, 0.3);
        slam.relocalization->set_gicp_param(use_gicp, gicp_downsample, search_radius, teps, feps, fitness_score);
    }

    vector<double> gravity;
    node->get_parameter_or("localization/gravity", gravity, vector<double>());

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
    rclcpp::Subscription<livox_interfaces2::msg::CustomMsg>::SharedPtr sub_pcl1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl2;
    if (lidar_type == AVIA)
        sub_pcl1 = node->create_subscription<livox_interfaces2::msg::CustomMsg>(lid_topic, 200000, livox_pcl_cbk);
    else
        sub_pcl2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000, standard_pcl_cbk);
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_cbk);
    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    auto pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
    // not used
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
    // not used
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
    auto pubImuPath = node->create_publisher<nav_msgs::msg::Path>("/imu_path", 100000);
    tf2_ros::TransformBroadcaster broadcaster(node);
    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    while (rclcpp::ok())
    {
        if (flg_exit)
            break;
        rclcpp::spin_some(node);

        if (!slam.sync_sensor_data())
            continue;

        if (slam.run())
        {
            const auto &state = slam.frontend->state;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, broadcaster, state, slam.frontend->kf, slam.lidar_end_time);

            /******* Publish points *******/
            if (path_en)
                publish_imu_path(pubImuPath, state, slam.lidar_end_time);
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
