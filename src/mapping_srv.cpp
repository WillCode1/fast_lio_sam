#include <ros/ros.h>
#include "fast_lio_sam/mapping.h"

#include <thread>
#include "system/DataDef.h"
#include "system/Header.h"
#include "system/Pcd2Pgm.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

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
        if (tmp_pc->points[i].z < -1.5 || tmp_pc->points[i].z > 0.1)
            continue;
        pcl::copyPoint(tmp_pc->points[i], keyframe_pc->points[i]);
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
    else if (request.action.compare("pcd2pgm") == 0)
    {
        ros::param::set("official/map_path", request.map_name);
        pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d(new pcl::PointCloud<PointXYZIRPYT>());
        PointCloudType::Ptr global_map(new PointCloudType());
        pcl::io::loadPCDFile(request.map_name + "/trajectory.pcd", *keyframe_pose6d);
        for (auto i = 0; i < keyframe_pose6d->size(); ++i)
        {
            PointCloudType::Ptr keyframe_pc(new PointCloudType());
            load_keyframe(request.map_name + "/keyframe/", keyframe_pc, i);
            octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
            *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d)[i]);
        }
        Pcd2Pgm mg(0.05, request.map_name + "/map");
        mg.convert_from_pcd(global_map);
        mg.convert_to_pgm();
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
    ROS_INFO("mapping service start!");

    ros::spin();
    return 0;
}
