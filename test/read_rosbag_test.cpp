#include <iostream>
#include <memory>
#include <csignal>
#include <unistd.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core.hpp>
#include "livox_ros_driver/CustomMsg.h"
#include "backend/backend/Backend.hpp"
#include "backend/utility/ProgressBar.h"
#include "ParametersRos1.h"
using namespace std;
FILE *location_log = nullptr;

bool save_globalmap_en;
const std::string root_path = std::string(ROOT_DIR);
std::string config_filename;
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

void standard_pcl_cbk(FastlioOdometry &frontend, const sensor_msgs::PointCloud2::ConstPtr &msg)
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
}

void livox_pcl_cbk(FastlioOdometry &frontend, const livox_ros_driver::CustomMsg::ConstPtr &msg)
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
}

void imu_cbk(FastlioOdometry &frontend, const sensor_msgs::Imu::ConstPtr &msg)
{
    frontend.cache_imu_data(msg->header.stamp.toSec(),
                            V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                            QD(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
}

void test_rosbag(const std::string &bagfile, const std::string &config_path, const std::vector<std::string> &topics, const std::string &bag_path)
{
    FastlioOdometry frontend;
    Backend backend;
    backend.globalmap_path = bag_path + "/globalmap.pcd";
    backend.trajectory_path = bag_path + "/trajectory.pcd";
    backend.keyframe_path = bag_path + "/keyframe/";
    backend.scd_path = bag_path + "/scancontext/";

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
    load_parameters(frontend, backend, save_globalmap_en, lidar_type);
    backend.test_mode = true;

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

        if (msg.isType<sensor_msgs::PointCloud2>())
        {
            sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
            standard_pcl_cbk(frontend, cloud);
            // if (frontend.sync_sensor_data())
            //     slam.run();
            printProgressBar(cost, bag_duration);
        }
        else if (msg.isType<livox_ros_driver::CustomMsg>())
        {
            livox_ros_driver::CustomMsg::ConstPtr cloud = msg.instantiate<livox_ros_driver::CustomMsg>();
            livox_pcl_cbk(frontend, cloud);
            // if (frontend.sync_sensor_data())
            //     slam.run();
            printProgressBar(cost, bag_duration);
        }
        else if (msg.isType<sensor_msgs::Imu>())
        {
            sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
            imu_cbk(frontend, imu);
            // if (frontend.sync_sensor_data())
            //     slam.run();
        }
    }

    bag.close();

    if (!flg_exit && !frontend.lidar_buffer.empty())
    {
        // slam.run();
    }
    backend.save_trajectory();

    if (save_globalmap_en)
        backend.save_globalmap();
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
            test_rosbag(entry.path(), config_path, topics, directoryPath);
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
        if (fs::is_regular_file(entry) && entry.path().filename().string() == config_filename)
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
    cv::FileStorage test_config(root_path + "/test/test_config.yaml", cv::FileStorage::READ);

    signal(SIGINT, SigHandle);

    config_filename = "mapping_dev.yaml";

    std::vector<std::string> dataset_paths;
    ros::param::param("config_file", topics, vector<std::string>());
    if (!test_config["read_topics"].empty())
        test_config["read_topics"] >> topics;
    if (!test_config["dataset_paths"].empty())
        test_config["dataset_paths"] >> dataset_paths;

    Timer timer;
    LOG_WARN("111");
    for (const auto& path: dataset_paths)
    {
        LOG_WARN("222");
        if (fs::exists(path) && fs::is_directory(path))
            traverse_for_config(path);
    }
    LOG_WARN("333");
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
