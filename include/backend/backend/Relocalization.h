#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "backend/Header.h"
#include "GnssProcessor.h"
#include "backend/global_localization/bnb3d.h"
#include "backend/global_localization/scancontext/Scancontext.h"
#define gnss_with_direction

class Relocalization
{
public:
    Relocalization();
    ~Relocalization();
    void set_extrinsic(const V3D &transl, const M3D &rot);
    bool load_prior_map(const PointCloudType::Ptr &global_map);
    bool load_keyframe_descriptor(const std::string &path);
    bool run(const PointCloudType::Ptr &scan, Eigen::Matrix4d &result, const double &lidar_beg_time);

    void set_init_pose(const Pose &_manual_pose);
    void set_bnb3d_param(const BnbOptions &match_option, const Pose &lidar_pose);
    void set_ndt_param(const double &_step_size, const double &_resolution);
    void set_gicp_param(bool _use_gicp, const double &filter_range, const double &gicp_ds, const double &search_radi, const double &tep, const double &fep, const double &fit_score);
    void add_keyframe_descriptor(const PointCloudType::Ptr thiskeyframe, const std::string &path);

    std::string algorithm_type = "UNKNOW";
    BnbOptions bnb_option;
    Pose manual_pose, lidar_extrinsic, rough_pose;
    std::shared_ptr<BranchAndBoundMatcher3D> bnb3d;

    pcl::PointCloud<PointXYZIRPYT>::Ptr trajectory_poses;
    std::shared_ptr<ScanContext::SCManager> sc_manager; // scan context

    GnssPose gnss_pose;
    Eigen::Matrix4d extrinsic_imu2gnss;

private:
    bool fine_tune_pose(PointCloudType::Ptr scan, Eigen::Matrix4d &result, const Eigen::Matrix4d &lidar_ext, const double &score);
    bool run_gnss_relocalization(PointCloudType::Ptr scan, Eigen::Matrix4d &result, const double &lidar_beg_time, const Eigen::Matrix4d &lidar_ext, double &score);
    bool run_scan_context(PointCloudType::Ptr scan, Eigen::Matrix4d &rough_mat, const Eigen::Matrix4d &lidar_ext, double &score);
    bool run_manually_set(PointCloudType::Ptr scan, Eigen::Matrix4d &rough_mat, const Eigen::Matrix4d &lidar_ext, double &score);

    bool prior_pose_inited = false;

    // ndt
    double step_size = 0.1;
    double resolution = 1;

    // gicp
    bool use_gicp = true;
    double filter_range = 80;
    double gicp_downsample = 0.2;
    double search_radius = 0.2;
    double teps = 0.001;
    double feps = 0.001;
    double fitness_score = 0.3;

    pcl::VoxelGrid<PointType> voxel_filter;
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
};
