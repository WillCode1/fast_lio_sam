#include <ros/ros.h>
#include "fast_lio_sam/mapping.h"

#include <thread>
#include "system/DataDef.h"
#include "system/Header.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>

double map_resolution = 0.05;
ros::Publisher map_topic_pub;

void load_keyframe(const std::string &keyframe_path, PointCloudType::Ptr keyframe_pc, int keyframe_cnt, int num_digits = 6)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt;
    std::string keyframe_idx = out.str();
    string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(keyframe_file, *tmp_pc);
    keyframe_pc->points.resize(tmp_pc->points.size());
    for (auto i = 0; i < tmp_pc->points.size(); ++i)
    {
        if (tmp_pc->points[i].z < -1.5 || tmp_pc->points[i].z > 1)
            continue;
        pcl::copyPoint(tmp_pc->points[i], keyframe_pc->points[i]);
    }
}

void SetMapTopicMsg(const PointCloudType::Ptr cloud, nav_msgs::OccupancyGrid &msg)
{
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = ros::Time::now();
    msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max; // 这里是投影到xy平面，如果要投到xz/yz，这里以及后面的xy对应的数据改为你想投影的平面

    if (cloud->points.empty())
    {
        ROS_WARN("pcd is empty!\n");

        return;
    }

    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }

        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if (x < x_min)
            x_min = x;
        if (x > x_max)
            x_max = x;

        if (y < y_min)
            y_min = y;
        if (y > y_max)
            y_max = y;
    }

    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);

    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    ROS_INFO("data size = %d\n", msg.data.size());

    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        int i = int((cloud->points[iter].x - x_min) / map_resolution);
        if (i < 0 || i >= msg.info.width)
            continue;

        int j = int((cloud->points[iter].y - y_min) / map_resolution);
        if (j < 0 || j >= msg.info.height - 1)
            continue;

        msg.data[i + j * msg.info.width] = 100;
    }
}

std::string execCommand(const std::string &command)
{
    std::array<char, 128> buffer;
    std::string result;

    ROS_INFO("exec \"%s\"", command.c_str());
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    return result;
}

std::string kill_progress_name = "";

bool mapping_callback(fast_lio_sam::mapping::Request &request, fast_lio_sam::mapping::Response &response)
{
    ROS_INFO("The slam request received. map name = %s, type = %s, action = %s.", request.map_name.c_str(), request.type.c_str(), request.action.c_str());

    if (request.type.compare("3d") != 0)
    {
        response.result = request.action + "_recv, but type error!";
        ROS_ERROR("type = %s, is error!", request.type.c_str());
        return true;
    }

    if (request.action.compare("start") == 0)
    {
        ros::param::set("official/map_path", request.map_name);
        kill_progress_name = "fastlio_sam_ros1";
        execCommand("gnome-terminal -- bash -c \"roslaunch fast_lio_sam run.launch\"");
    }
    else if (request.action.compare("stitch") == 0)
    {
        ros::param::set("official/prior_map_path", request.map1_name);
        ros::param::set("official/stitch_map_path", request.map2_name);
        ros::param::set("official/result_map_path", request.map_name);
        kill_progress_name = "map_stitch";
        execCommand("gnome-terminal -- bash -c \"roslaunch fast_lio_sam map_stitch.launch\"");
    }
    else if (request.action.compare("localization") == 0)
    {
        ros::param::set("official/map_path", request.map_name);
        kill_progress_name = "fastlio_localization_ros1";
        execCommand("gnome-terminal -- bash -c \"roslaunch fastlio_localization run_service.launch\"");
    }
    else if (request.action.compare("test") == 0)
    {
        ros::param::set("official/map_path", request.map_name);
        execCommand("gnome-terminal -- bash -c \"rosrun map_server map_saver\"");
        pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d(new pcl::PointCloud<PointXYZIRPYT>());
        PointCloudType::Ptr global_map(new PointCloudType());
        nav_msgs::OccupancyGrid map_topic_msg;
        pcl::io::loadPCDFile(request.map_name + "/trajectory.pcd", *keyframe_pose6d);
        for (auto i = 0; i < keyframe_pose6d->size(); ++i)
        {
            PointCloudType::Ptr keyframe_pc(new PointCloudType());
            load_keyframe(request.map_name + "/keyframe/", keyframe_pc, i);
            octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
            *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d)[i]);
        }
        SetMapTopicMsg(global_map, map_topic_msg);
        map_topic_pub.publish(map_topic_msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        execCommand("mv map.pgm " + request.map_name + "/map.pgm");
        execCommand("mv map.yaml " + request.map_name + "/map.yaml");
    }
    else if (kill_progress_name.compare("") == 0)
    {
        response.result = request.action + "_recv";
        return true;
    }
    else if (request.action.compare("finish") == 0)
    {
        execCommand("gnome-terminal -- bash -c \"killall -2 " + kill_progress_name + "\"");
        // execCommand("gnome-terminal -- bash -c \"killall -2 gnome-terminal-\"");
    }
    else if (request.action.compare("cancel") == 0)
    {
        execCommand("gnome-terminal -- bash -c \"killall -2 " + kill_progress_name + "\"");
    }

    response.result = request.action + "_recv";
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"mapping_srv");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("mappingSrv", mapping_callback);
    map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    ROS_INFO("mapping service start!");

    ros::spin();
    return 0;
}
