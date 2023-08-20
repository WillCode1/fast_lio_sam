#include <iostream>
#include <memory>
#include <experimental/filesystem>
#include <csignal>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include "livox_ros_driver/CustomMsg.h"
#include "system/System.hpp"
#include "utility/ProgressBar.h"
using namespace std;
namespace fs = std::experimental::filesystem;

bool save_globalmap_en, pure_localization;
const std::string root_path = std::string(ROOT_DIR);
std::string relocal_config_path;
std::string dataset_path;
std::vector<std::string> topics;

std::set<std::string> test_bag_name;
double bag_total_time = 0;
double test_cost_total_time = 0;

int lidar_type;
bool flg_exit = false;
void SigHandle(int sig)
{
    flg_exit = true;
    LOG_WARN("catch sig %d", sig);
}

void standard_pcl_cbk(System& slam, const sensor_msgs::PointCloud2::ConstPtr &msg)
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

void livox_pcl_cbk(System& slam, const livox_ros_driver::CustomMsg::ConstPtr &msg)
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

void imu_cbk(System& slam, const sensor_msgs::Imu::ConstPtr &msg)
{
    slam.cache_imu_data(msg->header.stamp.toSec(),
                        V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), 
                        V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
}

void load_config(System& slam, const std::string &config_path)
{
    char ws_path[PATH_MAX];
    getcwd(ws_path, sizeof(ws_path));

    YAML::Node config = YAML::LoadFile(config_path);
    YAML::Node relocal_config = YAML::LoadFile(root_path + relocal_config_path);

    double blind, detect_range;
    int n_scans, scan_rate, time_unit;
    vector<double> extrinT;
    vector<double> extrinR;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    string lid_topic, imu_topic;

    slam.frontend->num_max_iterations = config["mapping"]["max_iteration"].IsDefined() ? config["mapping"]["max_iteration"].as<int>() : 4;
    lid_topic = config["common"]["lid_topic"].IsDefined() ? config["common"]["lid_topic"].as<string>() : "/livox/lidar";
    imu_topic = config["common"]["imu_topic"].IsDefined() ? config["common"]["imu_topic"].as<string>() : "/livox/imu";
    slam.timedelay_lidar2imu = config["common"]["timedelay_lidar2imu"].IsDefined() ? config["common"]["timedelay_lidar2imu"].as<double>() : 0;
    slam.frontend->surf_frame_ds_res = config["mapping"]["surf_frame_ds_res"].IsDefined() ? config["mapping"]["surf_frame_ds_res"].as<double>() : 0.5;
    slam.frontend->point_skip_num = config["mapping"]["point_skip_num"].IsDefined() ? config["mapping"]["point_skip_num"].as<int>() : 2;
    slam.frontend->ikdtree_resolution = config["mapping"]["ikdtree_resolution"].IsDefined() ? config["mapping"]["ikdtree_resolution"].as<double>() : 0.5;
    slam.frontend->cube_len = config["mapping"]["cube_side_length"].IsDefined() ? config["mapping"]["cube_side_length"].as<double>() : 200;

    slam.backend->keyframe_add_dist_threshold = config["mapping"]["keyframe_add_dist_threshold"].IsDefined() ? config["mapping"]["keyframe_add_dist_threshold"].as<float>() : 1;
    slam.backend->keyframe_add_angle_threshold = config["mapping"]["keyframe_add_angle_threshold"].IsDefined() ? config["mapping"]["keyframe_add_angle_threshold"].as<float>() : 0.2;
    slam.backend->pose_cov_threshold = config["mapping"]["pose_cov_threshold"].IsDefined() ? config["mapping"]["pose_cov_threshold"].as<float>() : 25;
    slam.backend->recontruct_kdtree = config["mapping"]["recontruct_kdtree"].IsDefined() ? config["mapping"]["recontruct_kdtree"].as<bool>() : true;
    slam.backend->ikdtree_reconstruct_keyframe_num = config["mapping"]["ikdtree_reconstruct_keyframe_num"].IsDefined() ? config["mapping"]["ikdtree_reconstruct_keyframe_num"].as<float>() : 10;
    slam.backend->ikdtree_reconstruct_downsamp_size = config["mapping"]["ikdtree_reconstruct_downsamp_size"].IsDefined() ? config["mapping"]["ikdtree_reconstruct_downsamp_size"].as<float>() : 0.1;

    slam.loop_closure_enable_flag = config["mapping"]["loop_closure_enable_flag"].IsDefined() ? config["mapping"]["loop_closure_enable_flag"].as<bool>() : false;
    slam.loopClosure->manually_fine_tune_loop_closure = config["mapping"]["manually_fine_tune_loop_closure"].IsDefined() ? config["mapping"]["manually_fine_tune_loop_closure"].as<bool>() : false;
    slam.loopClosure->loop_keyframe_num_thld = config["mapping"]["loop_keyframe_num_thld"].IsDefined() ? config["mapping"]["loop_keyframe_num_thld"].as<int>() : 50;
    slam.loopClosure->loop_closure_search_radius = config["mapping"]["loop_closure_search_radius"].IsDefined() ? config["mapping"]["loop_closure_search_radius"].as<float>() : 10;
    slam.loopClosure->loop_closure_search_time_interval = config["mapping"]["loop_closure_search_time_interval"].IsDefined() ? config["mapping"]["loop_closure_search_time_interval"].as<float>() : 30;
    slam.loopClosure->keyframe_search_num = config["mapping"]["keyframe_search_num"].IsDefined() ? config["mapping"]["keyframe_search_num"].as<int>() : 20;
    slam.loopClosure->loop_closure_fitness_score_thld = config["mapping"]["loop_closure_fitness_score_thld"].IsDefined() ? config["mapping"]["loop_closure_fitness_score_thld"].as<float>() : 0.05;
    slam.loopClosure->icp_downsamp_size = config["mapping"]["icp_downsamp_size"].IsDefined() ? config["mapping"]["icp_downsamp_size"].as<float>() : 0.1;
    slam.loopClosure->loop_closure_vaild_time_period = config["mapping"]["loop_closure_vaild_time_period"].IsDefined() ? config["mapping"]["loop_closure_vaild_time_period"].as<vector<double>>() : vector<double>();

    gyr_cov = config["mapping"]["gyr_cov"].IsDefined() ? config["mapping"]["gyr_cov"].as<double>() : 0.1;
    acc_cov = config["mapping"]["acc_cov"].IsDefined() ? config["mapping"]["acc_cov"].as<double>() : 0.1;
    b_gyr_cov = config["mapping"]["b_gyr_cov"].IsDefined() ? config["mapping"]["b_gyr_cov"].as<double>() : 0.0001;
    b_acc_cov = config["mapping"]["b_acc_cov"].IsDefined() ? config["mapping"]["b_acc_cov"].as<double>() : 0.0001;
    blind = config["preprocess"]["blind"].IsDefined() ? config["preprocess"]["blind"].as<double>() : 0.01;
    detect_range = config["preprocess"]["det_range"].IsDefined() ? config["preprocess"]["det_range"].as<double>() : 300.f;
    lidar_type = config["preprocess"]["lidar_type"].IsDefined() ? config["preprocess"]["lidar_type"].as<int>() : AVIA;
    n_scans = config["preprocess"]["scan_line"].IsDefined() ? config["preprocess"]["scan_line"].as<int>() : 16;
    time_unit = config["preprocess"]["timestamp_unit"].IsDefined() ? config["preprocess"]["timestamp_unit"].as<int>() : US;
    scan_rate = config["preprocess"]["scan_rate"].IsDefined() ? config["preprocess"]["scan_rate"].as<int>() : 10;
    slam.loger.runtime_log = config["mapping"]["runtime_log_enable"].IsDefined() ? config["mapping"]["runtime_log_enable"].as<int>() : 0;
    slam.frontend->extrinsic_est_en = config["mapping"]["extrinsic_est_en"].IsDefined() ? config["mapping"]["extrinsic_est_en"].as<bool>() : true;
    save_globalmap_en = config["official"]["save_globalmap_en"].IsDefined() ? config["official"]["save_globalmap_en"].as<bool>() : true;
    slam.save_keyframe_en = config["official"]["save_keyframe_en"].IsDefined() ? config["official"]["save_keyframe_en"].as<bool>() : true;
    slam.save_resolution = config["official"]["save_resolution"].IsDefined() ? config["official"]["save_resolution"].as<float>() : 0.1;
    extrinT = config["mapping"]["extrinsic_T"].IsDefined() ? config["mapping"]["extrinsic_T"].as<vector<double>>() : vector<double>();
    extrinR = config["mapping"]["extrinsic_R"].IsDefined() ? config["mapping"]["extrinsic_R"].as<vector<double>>() : vector<double>();

    BnbOptions match_option;
    Pose init_pose, lidar_pose;
    match_option.algorithm_type = relocal_config["bnb3d"]["algorithm_type"].IsDefined() ? relocal_config["bnb3d"]["algorithm_type"].as<string>() : std::string("UNKONW");
    match_option.linear_xy_window_size = relocal_config["bnb3d"]["linear_xy_window_size"].IsDefined() ? relocal_config["bnb3d"]["linear_xy_window_size"].as<double>() : 10;
    match_option.linear_z_window_size = relocal_config["bnb3d"]["linear_z_window_size"].IsDefined() ? relocal_config["bnb3d"]["linear_z_window_size"].as<double>() : 1.;
    match_option.angular_search_window = relocal_config["bnb3d"]["angular_search_window"].IsDefined() ? relocal_config["bnb3d"]["angular_search_window"].as<double>() : 30;
    match_option.pc_resolutions = relocal_config["bnb3d"]["pc_resolutions"].IsDefined() ? relocal_config["bnb3d"]["pc_resolutions"].as<vector<double>>() : vector<double>();
    match_option.bnb_depth = relocal_config["bnb3d"]["bnb_depth"].IsDefined() ? relocal_config["bnb3d"]["bnb_depth"].as<int>() : 5;
    match_option.min_score = relocal_config["bnb3d"]["min_score"].IsDefined() ? relocal_config["bnb3d"]["min_score"].as<double>() : 0.1;
    match_option.min_xy_resolution = relocal_config["bnb3d"]["min_xy_resolution"].IsDefined() ? relocal_config["bnb3d"]["min_xy_resolution"].as<double>() : 0.2;
    match_option.min_z_resolution = relocal_config["bnb3d"]["min_z_resolution"].IsDefined() ? relocal_config["bnb3d"]["min_z_resolution"].as<double>() : 0.1;
    match_option.min_angular_resolution = relocal_config["bnb3d"]["min_angular_resolution"].IsDefined() ? relocal_config["bnb3d"]["min_angular_resolution"].as<double>() : 0.1;
    match_option.thread_num = relocal_config["bnb3d"]["thread_num"].IsDefined() ? relocal_config["bnb3d"]["thread_num"].as<int>() : 4;
    match_option.filter_size_scan = relocal_config["bnb3d"]["filter_size_scan"].IsDefined() ? relocal_config["bnb3d"]["filter_size_scan"].as<double>() : 0.1;
    match_option.debug_mode = relocal_config["bnb3d"]["debug_mode"].IsDefined() ? relocal_config["bnb3d"]["debug_mode"].as<bool>() : false;

    slam.relocalization->need_wait_prior_pose_inited = relocal_config["bnb3d"]["need_wait_prior_pose_inited"].IsDefined() ? relocal_config["bnb3d"]["need_wait_prior_pose_inited"].as<bool>() : true;
    init_pose.x = relocal_config["bnb3d"]["init_pose"]["x"].IsDefined() ? relocal_config["bnb3d"]["init_pose"]["x"].as<double>() : 0.;
    init_pose.y = relocal_config["bnb3d"]["init_pose"]["y"].IsDefined() ? relocal_config["bnb3d"]["init_pose"]["y"].as<double>() : 0.;
    init_pose.z = relocal_config["bnb3d"]["init_pose"]["z"].IsDefined() ? relocal_config["bnb3d"]["init_pose"]["z"].as<double>() : 0.;
    init_pose.roll = relocal_config["bnb3d"]["init_pose"]["roll"].IsDefined() ? relocal_config["bnb3d"]["init_pose"]["roll"].as<double>() : 0.;
    init_pose.pitch = relocal_config["bnb3d"]["init_pose"]["pitch"].IsDefined() ? relocal_config["bnb3d"]["init_pose"]["pitch"].as<double>() : 0.;
    init_pose.yaw = relocal_config["bnb3d"]["init_pose"]["yaw"].IsDefined() ? relocal_config["bnb3d"]["init_pose"]["yaw"].as<double>() : 0.;

    lidar_pose.x = relocal_config["bnb3d"]["lidar_ext"]["x"].IsDefined() ? relocal_config["bnb3d"]["lidar_ext"]["x"].as<double>() : 0.;
    lidar_pose.y = relocal_config["bnb3d"]["lidar_ext"]["y"].IsDefined() ? relocal_config["bnb3d"]["lidar_ext"]["y"].as<double>() : 0.;
    lidar_pose.z = relocal_config["bnb3d"]["lidar_ext"]["z"].IsDefined() ? relocal_config["bnb3d"]["lidar_ext"]["z"].as<double>() : 0.;
    lidar_pose.roll = relocal_config["bnb3d"]["lidar_ext"]["roll"].IsDefined() ? relocal_config["bnb3d"]["lidar_ext"]["roll"].as<double>() : 0.;
    lidar_pose.pitch = relocal_config["bnb3d"]["lidar_ext"]["pitch"].IsDefined() ? relocal_config["bnb3d"]["lidar_ext"]["pitch"].as<double>() : 0.;
    lidar_pose.yaw = relocal_config["bnb3d"]["lidar_ext"]["yaw"].IsDefined() ? relocal_config["bnb3d"]["lidar_ext"]["yaw"].as<double>() : 0.;
    slam.relocalization->set_bnb3d_param(match_option, init_pose, lidar_pose);

    int min_plane_point;
    double filter_radius, cluster_dis, plane_dis, plane_point_percent;
    filter_radius = relocal_config["gicp"]["filter_radius"].IsDefined() ? relocal_config["gicp"]["filter_radius"].as<double>() : 1;
    min_plane_point = relocal_config["gicp"]["min_plane_point"].IsDefined() ? relocal_config["gicp"]["min_plane_point"].as<int>() : 20;
    cluster_dis = relocal_config["gicp"]["cluster_dis"].IsDefined() ? relocal_config["gicp"]["cluster_dis"].as<double>() : 0.1;
    plane_dis = relocal_config["gicp"]["plane_dis"].IsDefined() ? relocal_config["gicp"]["plane_dis"].as<double>() : 0.1;
    plane_point_percent = relocal_config["gicp"]["plane_point_percent"].IsDefined() ? relocal_config["gicp"]["plane_point_percent"].as<double>() : 0.1;
    slam.relocalization->set_plane_extract_param(filter_radius, min_plane_point, cluster_dis, plane_dis, plane_point_percent);

    double gicp_downsample, search_radius, teps, feps, fitness_score;
    gicp_downsample = relocal_config["gicp"]["gicp_downsample"].IsDefined() ? relocal_config["gicp"]["gicp_downsample"].as<double>() : 0.2;
    search_radius = relocal_config["gicp"]["search_radius"].IsDefined() ? relocal_config["gicp"]["search_radius"].as<double>() : 0.5;
    teps = relocal_config["gicp"]["teps"].IsDefined() ? relocal_config["gicp"]["teps"].as<double>() : 1e-3;
    feps = relocal_config["gicp"]["feps"].IsDefined() ? relocal_config["gicp"]["feps"].as<double>() : 1e-3;
    fitness_score = relocal_config["gicp"]["fitness_score"].IsDefined() ? relocal_config["gicp"]["fitness_score"].as<double>() : 0.3;
    slam.relocalization->set_gicp_param(gicp_downsample, search_radius, teps, feps, fitness_score);

    vector<double> gravity;
    gravity = config["localization"]["gravity"].IsDefined() ? config["localization"]["gravity"].as<vector<double>>() : vector<double>();

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
}

void test_rosbag(const std::string &bagfile, const std::string &config_path, const std::vector<std::string> &topics)
{
    System slam;
    rosbag::Bag bag;

    try
    {
        bag.open(bagfile, rosbag::bagmode::Read);
    }
    catch(const std::exception& e)
    {
        std::cout << bagfile << '\n';
        std::cerr << e.what() << '\n';
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    load_config(slam, config_path);

    ros::Time start_time = view.getBeginTime();
    ros::Time end_time = view.getEndTime();
    double bag_duration = (end_time - start_time).toSec();
    bag_total_time += bag_duration;

    LOG_INFO("Test rosbag %s...", bagfile.c_str());
    printProgressBar(0, bag_duration);

    for (const rosbag::MessageInstance& msg : view)
    {
        if (flg_exit)
            break;

        const auto &cost = (msg.getTime() - start_time).toSec();
        // if (cost / bag_duration < 0.98)
            // continue;

        if (msg.isType<sensor_msgs::PointCloud2>())
        {
            sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
            standard_pcl_cbk(slam, cloud);
            slam.run();
            printProgressBar(cost, bag_duration);
        }
        else if (msg.isType<livox_ros_driver::CustomMsg>())
        {
            livox_ros_driver::CustomMsg::ConstPtr cloud = msg.instantiate<livox_ros_driver::CustomMsg>();
            livox_pcl_cbk(slam, cloud);
            slam.run();
            printProgressBar(cost, bag_duration);
        }
        else if (msg.isType<sensor_msgs::Imu>())
        {
            sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
            imu_cbk(slam, imu);
            slam.run();
        }
    }

    bag.close();

    if (!flg_exit && !slam.lidar->lidar_buffer.empty())
    {
        slam.run();
    }
    slam.save_trajectory();

    if (save_globalmap_en)
        slam.save_globalmap();
}

std::string execCommand(const std::string &command)
{
    std::array<char, 128> buffer;
    std::string result;

    LOG_INFO("exec \"%s\"", command.c_str());
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    return result;
}

void traverse_for_testbag(const std::string& config_path, const std::string &directoryPath)
{
    for (const auto &entry : fs::directory_iterator(directoryPath))
    {
        if (flg_exit)
            break;
        else if (fs::is_regular_file(entry) && entry.path().filename().extension() == ".bag")
        {
            // std::cout << entry.path().filename() << std::endl;
            test_rosbag(entry.path(), config_path, topics);
            execCommand("cp " + DEBUG_FILE_DIR("keyframe_pose.txt") + " " + directoryPath);
            execCommand("cp " + DEBUG_FILE_DIR("keyframe_pose_optimized.txt") + " " + directoryPath);
            test_bag_name.insert(entry.path().filename());
        }
        else if (fs::is_directory(entry))
        {
            traverse_for_testbag(config_path, entry.path().string());
        }
    }
}

void traverse_for_config(const std::string &directoryPath)
{
    for (const auto &entry : fs::directory_iterator(directoryPath))
    {
        if (flg_exit)
            break;
        if (fs::is_regular_file(entry) && entry.path().filename().extension() == ".yaml")
        {
            traverse_for_testbag(entry.path().string(), directoryPath);
        }
        else if (fs::is_directory(entry))
        {
            traverse_for_config(entry.path().string());
        }
    }
}

int main(int argc, char** argv)
{
    YAML::Node test_config = YAML::LoadFile(root_path + "/test/test_config.yaml");

    signal(SIGINT, SigHandle);
    pure_localization = test_config["pure_localization"].IsDefined() ? test_config["pure_localization"].as<bool>() : false;

    relocal_config_path = test_config["relocal_config"].IsDefined() ? test_config["relocal_config"].as<std::string>() : std::string("");

    topics = test_config["read_topics"].IsDefined() ? test_config["read_topics"].as<vector<std::string>>() : vector<std::string>();

    Timer timer;
    auto dataset_paths = test_config["dataset_paths"].IsDefined() ? test_config["dataset_paths"].as<vector<std::string>>() : vector<std::string>();
    for (const auto& path: dataset_paths)
    {
        if (fs::exists(path) && fs::is_directory(path))
            traverse_for_config(path);
    }
    test_cost_total_time += timer.elapsedStart() / 1000;

    LOG_WARN("=================total test bag=================");
    for (const auto& name: test_bag_name)
    {
        std::cout << name << std::endl;
    }
    LOG_WARN("test_bag_num = %lu, bag_total_time = %.3lf, test_cost_total_time = %.3lf!",
             test_bag_name.size(), bag_total_time, test_cost_total_time);

    return 0;
}
