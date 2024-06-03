#include <csignal>
#include <ros/ros.h>
#include "system/MapStitch.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

double globalMapVisualizationPoseDensity = 10;
double globalMapVisualizationLeafSize = 1;
std::string map_frame = "map";

FILE *location_log = nullptr;
bool flg_exit = false;
void SigHandle(int sig)
{
    flg_exit = true;
    LOG_WARN("catch sig %d", sig);
}

void publish_cloud(const ros::Publisher &pubCloud, PointCloudType::Ptr cloud, const double& lidar_end_time, const std::string& frame_id)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
    cloud_msg.header.frame_id = frame_id;
    pubCloud.publish(cloud_msg);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_stitch");
    ros::NodeHandle nh;
    MapStitch map_stitch;

    ros::param::param("mapping/loop_keyframe_num_thld", map_stitch.loop_keyframe_num_thld, 50);
    ros::param::param("mapping/loop_closure_search_radius", map_stitch.loop_closure_search_radius, 10.f);
    ros::param::param("mapping/keyframe_search_num", map_stitch.keyframe_search_num, 20);
    ros::param::param("mapping/loop_closure_fitness_score_thld", map_stitch.loop_closure_fitness_score_thld, 0.05f);
    ros::param::param("mapping/icp_downsamp_size", map_stitch.icp_downsamp_size, 0.1f);
    ros::param::param("mapping/manually_loop_vaild_period", map_stitch.loop_vaild_period["manually"], vector<double>());
    ros::param::param("mapping/odom_loop_vaild_period", map_stitch.loop_vaild_period["odom"], vector<double>());
    ros::param::param("mapping/scancontext_loop_vaild_period", map_stitch.loop_vaild_period["scancontext"], vector<double>());

    ros::param::param("scan_context/lidar_height", map_stitch.relocalization->sc_manager->LIDAR_HEIGHT, 2.0);
    ros::param::param("scan_context/sc_dist_thres", map_stitch.relocalization->sc_manager->SC_DIST_THRES, 0.5);

    ros::param::param("utm_origin/zone", map_stitch.relocalization->utm_origin.zone, std::string("51N"));
    ros::param::param("utm_origin/east", map_stitch.relocalization->utm_origin.east, 0.);
    ros::param::param("utm_origin/north", map_stitch.relocalization->utm_origin.north, 0.);
    ros::param::param("utm_origin/up", map_stitch.relocalization->utm_origin.up, 0.);

    vector<double> extrinT;
    vector<double> extrinR;
    V3D extrinT_eigen;
    M3D extrinR_eigen;
    ros::param::param("mapping/extrinsicT_imu2gnss", extrinT, vector<double>());
    ros::param::param("mapping/extrinsicR_imu2gnss", extrinR, vector<double>());
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    map_stitch.relocalization->set_extrinsic(extrinT_eigen, extrinR_eigen);

    ros::param::param("relocalization_cfg/algorithm_type", map_stitch.relocalization->algorithm_type, std::string("UNKONW"));

    BnbOptions match_option;
    ros::param::param("bnb3d/linear_xy_window_size", match_option.linear_xy_window_size, 10.);
    ros::param::param("bnb3d/linear_z_window_size", match_option.linear_z_window_size, 1.);
    ros::param::param("bnb3d/angular_search_window", match_option.angular_search_window, 30.);
    ros::param::param("bnb3d/pc_resolutions", match_option.pc_resolutions, vector<double>());
    ros::param::param("bnb3d/bnb_depth", match_option.bnb_depth, 5);
    ros::param::param("bnb3d/min_score", match_option.min_score, 0.1);
    ros::param::param("bnb3d/enough_score", match_option.enough_score, 0.8);
    ros::param::param("bnb3d/min_xy_resolution", match_option.min_xy_resolution, 0.2);
    ros::param::param("bnb3d/min_z_resolution", match_option.min_z_resolution, 0.1);
    ros::param::param("bnb3d/min_angular_resolution", match_option.min_angular_resolution, 0.1);
    ros::param::param("bnb3d/thread_num", match_option.thread_num, 4);
    ros::param::param("bnb3d/filter_size_scan", match_option.filter_size_scan, 0.1);
    ros::param::param("bnb3d/debug_mode", match_option.debug_mode, false);

    Pose lidar_extrinsic;
    ros::param::param("relocalization_cfg/lidar_ext/x", lidar_extrinsic.x, 0.);
    ros::param::param("relocalization_cfg/lidar_ext/y", lidar_extrinsic.y, 0.);
    ros::param::param("relocalization_cfg/lidar_ext/z", lidar_extrinsic.z, 0.);
    ros::param::param("relocalization_cfg/lidar_ext/roll", lidar_extrinsic.roll, 0.);
    ros::param::param("relocalization_cfg/lidar_ext/pitch", lidar_extrinsic.pitch, 0.);
    ros::param::param("relocalization_cfg/lidar_ext/yaw", lidar_extrinsic.yaw, 0.);
    map_stitch.relocalization->set_bnb3d_param(match_option, lidar_extrinsic);

    double step_size, resolution;
    ros::param::param("ndt/step_size", step_size, 0.1);
    ros::param::param("ndt/resolution", resolution, 1.);
    map_stitch.relocalization->set_ndt_param(step_size, resolution);

    bool use_gicp;
    double gicp_downsample, filter_range, search_radius, teps, feps, fitness_score;
    ros::param::param("gicp/use_gicp", use_gicp, true);
    ros::param::param("gicp/filter_range", filter_range, 80.);
    ros::param::param("gicp/gicp_downsample", gicp_downsample, 0.2);
    ros::param::param("gicp/search_radius", search_radius, 0.5);
    ros::param::param("gicp/teps", teps, 1e-3);
    ros::param::param("gicp/feps", feps, 1e-3);
    ros::param::param("gicp/fitness_score", fitness_score, 0.3);
    map_stitch.relocalization->set_gicp_param(use_gicp, filter_range, gicp_downsample, search_radius, teps, feps, fitness_score);

    ros::param::param("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.);
    ros::param::param("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.);

    map_stitch.load_prior_map_info("/home/will/data/test_mapping/mapping1");
    map_stitch.load_stitch_map_info("/home/will/data/test_mapping/mapping2");

    ros::Publisher pubPriorMap = nh.advertise<sensor_msgs::PointCloud2>("/map_prior", 1);
    ros::Publisher pubStitchMap = nh.advertise<sensor_msgs::PointCloud2>("/map_stitch", 1);
    ros::Publisher pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);

    signal(SIGINT, SigHandle);
    ros::Rate rate(1);
    while (ros::ok())
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        auto prior_map_visual = map_stitch.get_map_visual(globalMapVisualizationPoseDensity, globalMapVisualizationLeafSize, map_stitch.keyframe_pose6d_prior, map_stitch.keyframe_scan_prior);
        if (prior_map_visual)
        {
            publish_cloud(pubPriorMap, prior_map_visual, 10000, map_frame);
        }

        auto stitch_map_visual = map_stitch.get_map_visual(globalMapVisualizationPoseDensity, globalMapVisualizationLeafSize, map_stitch.keyframe_pose6d_stitch, map_stitch.keyframe_scan_stitch);
        if (stitch_map_visual)
        {
            publish_cloud(pubStitchMap, stitch_map_visual, 10000, map_frame);
        }

        visualize_loop_closure_constraints(pubLoopConstraintEdge, 10000, map_stitch.loop_constraint_records, map_stitch.keyframe_pose6d_optimized);
        rate.sleep();
    }
    return 0;
}
