#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include "../Header.h"
#include "FactorGraphOptimization.h"
#include "Relocalization.h"
#include "LoopClosure.h"

class MapStitch
{
public:
    MapStitch();

    void load_prior_map_info(const std::string &path);

    void load_stitch_map_info(const std::string &path);

    void save_results_info(const std::string &path);

    PointCloudType::Ptr get_map_visual(float globalMapVisualizationPoseDensity, float globalMapVisualizationLeafSize,
                                       const pcl::PointCloud<PointXYZIRPYT>::Ptr &keyframe_pose, const deque<PointCloudType::Ptr> &keyframe_scan);

private:
    /**
     * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
     */
    void loop_find_near_keyframes(PointCloudType::Ptr &near_keyframes, const int &key, const int &search_num,
                                  const deque<PointCloudType::Ptr> &keyframe_scan);

    void perform_loop_closure(int loop_key_cur, int loop_key_ref,
                              const std::string &type, bool use_guess = false, const Eigen::Matrix4f &init_guess = Eigen::Matrix4f::Identity());

    void detect_loop_by_distance(int index);

    void detect_loop_by_scancontext(int index);

    void run_loop(int index);

    bool is_vaild_loop_time_period(const double &time, const std::vector<double> &vaild_period);

    void load_keyframe(const std::string &keyframe_path, PointCloudType::Ptr keyframe_pc, int keyframe_cnt, int num_digits = 6);

    bool load_keyframe_descriptor(const std::string &path);

    void load_factor_graph(const std::string &path, int index_offset = 0);

    void save_factor_graph(const std::string &map_path);

    void save_keyframe(PointCloudType::Ptr scan, const std::string &keyframe_path, int keyframe_cnt, int num_digits = 6);

    void saveSCD(const Eigen::MatrixXd &scd, int index, const std::string &save_path, int num_digits = 6, const std::string &delimiter = " ");

    void save_globalmap(const std::string &globalmap_path, const double &save_resolution);

public:
    double dartion_time;
    std::unordered_map<std::string, std::vector<double>> loop_vaild_period;
    int loop_keyframe_num_thld = 50;
    float loop_closure_search_radius = 10;
    int keyframe_search_num = 20;
    float loop_closure_fitness_score_thld = 0.05;
    float icp_downsamp_size = 0.1;

    pcl::KdTreeFLANN<PointXYZIRPYT>::Ptr kdtree_history_keyframe_pose;

    LoopConstraint loop_constraint;

    deque<PointCloudType::Ptr> keyframe_scan_prior;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_prior;
    shared_ptr<Relocalization> relocalization;

    deque<PointCloudType::Ptr> keyframe_scan_stitch;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_stitch;
    std::shared_ptr<ScanContext::SCManager> sc_manager_stitch;

    unordered_map<int, int> loop_constraint_records_prior;
    unordered_map<int, int> loop_constraint_records_stitch;
    unordered_map<int, int> loop_constraint_records_add;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_optimized;

    // gtsam
    gtsam::NonlinearFactorGraph gtsam_graph;
    gtsam::Values init_estimate;
    gtsam::Values optimized_estimate;
    gtsam::ISAM2 *isam;

    std::map<int, gtsam::Pose3> init_values;
    std::priority_queue<GtsamFactor> gtsam_factors;

    bool save_globalmap_en;
    float save_resolution;
};
