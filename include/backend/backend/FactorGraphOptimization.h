#pragma once
#include <pcl/filters/voxel_grid.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h>

#include <map>
#include <queue>
#include "backend/Header.h"
#include "GnssProcessor.h"

#define MAP_STITCH

#ifdef MAP_STITCH
struct GtsamFactor
{
    enum FactorType
    {
        Prior,
        Between,
        Loop,
        Gps
    };

    bool operator<(const GtsamFactor &y) const;

    FactorType factor_type;
    int index_from;
    int index_to;
    gtsam::Pose3 value;
    gtsam::Vector noise;
};
#endif

class FactorGraphOptimization
{
public:
    FactorGraphOptimization(const pcl::PointCloud<PointXYZIRPYT>::Ptr &keyframe_pose,
                            const shared_ptr<deque<PointCloudType::Ptr>> &keyframe_cloud,
                            const shared_ptr<GnssProcessor> &p_gnss);

    bool is_keyframe(const PointXYZIRPYT &this_pose6d);

    void run(LoopConstraint &loop_constraint, PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &submap_fix);

    void get_keyframe_pose6d(pcl::PointCloud<PointXYZIRPYT>::Ptr& copy_keyframe_pose6d);

private:
    void add_odom_factor(const PointXYZIRPYT &this_pose6d);

    void add_gnss_factor(const PointXYZIRPYT &this_pose6d);

    void add_loop_factor(LoopConstraint &loop_constraint);

    void add_height_factor(PointXYZIRPYT this_pose6d);

    void add_factor_and_optimize(LoopConstraint &loop_constraint, PointXYZIRPYT &this_pose6d);

    void get_submap_fix(PointCloudType::Ptr &submap_fix);

    void correct_poses(PointCloudType::Ptr &submap_fix);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::mutex pose_mtx;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_optimized;
    shared_ptr<deque<PointCloudType::Ptr>> keyframe_scan;

    /* loop clousre */
    float pose_cov_threshold = 25;
    std::vector<double> gnss_weight;
    shared_ptr<GnssProcessor> gnss;
    bool loop_is_closed = false;
    bool z_axis_is_constrained = false;

    // gtsam
    gtsam::NonlinearFactorGraph gtsam_graph;
    gtsam::Values init_estimate;
    gtsam::Values optimized_estimate;
    gtsam::ISAM2 *isam;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise;
    Eigen::MatrixXd pose_covariance;

    // key frame param
    float keyframe_add_dist_threshold = 1;      // m
    float keyframe_add_angle_threshold = 0.2;   // 11.46 degree

    // ikdtree reconstruct
    bool recontruct_kdtree = true;
    int ikdtree_reconstruct_keyframe_num = 10;
    float ikdtree_reconstruct_downsamp_size = 0.1;

#ifdef MAP_STITCH
    std::map<int, gtsam::Pose3> init_values;
    std::queue<GtsamFactor> gtsam_factors;
#endif

    bool z_axis_constraint_enable = false;
    bool update_height = true;
    double cur_height = -10000000;
    double add_height_factor_threshold = 0.1;
};
