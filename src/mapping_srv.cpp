#include <ros/ros.h>
#include "fast_lio_sam/mapping.h"

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
        kill_progress_name = "fastlio_sam_ros";
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
