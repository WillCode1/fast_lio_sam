#pragma once
#include <unordered_map>
#include <condition_variable>
#include <atomic>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include "backend/Header.h"
#ifdef USE_PANGOLIN
#include "backend/utility/manually_correct_loop_closure.h"
#endif
#include "backend/global_localization/scancontext/Scancontext.h"

class LoopClosure
{
public:
    LoopClosure(const std::shared_ptr<ScanContext::SCManager> scManager, std::condition_variable &cv, std::atomic<bool> &isABlocked);

    /**
     * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
     */
    void loop_find_near_keyframes(PointCloudType::Ptr &near_keyframes, const int &key, const int &search_num,
                                  const deque<PointCloudType::Ptr> &keyframe_scan);

    void perform_loop_closure(const deque<PointCloudType::Ptr> &keyframe_scan, int loop_key_cur, int loop_key_ref,
                              const std::string &type, bool use_guess = false, const Eigen::Matrix4f &init_guess = Eigen::Matrix4f::Identity());

    void detect_loop_by_distance(const deque<PointCloudType::Ptr> &keyframe_scan);

    void detect_loop_by_scancontext(const deque<PointCloudType::Ptr> &keyframe_scan);

    void run(const deque<PointCloudType::Ptr> &keyframe_scan);

    void get_loop_constraint(LoopConstraint &loop_constr);

    bool is_vaild_loop_time_period(const double &time, const std::vector<double> &vaild_period);

public:
    std::condition_variable &cv;
    std::atomic<bool> &isABlocked;

    std::unordered_map<std::string, std::vector<double>> loop_vaild_period;
    std::mutex loop_mtx;
    int loop_keyframe_num_thld = 50;
    float loop_closure_search_radius = 10;
    int loop_closure_keyframe_interval = 30;
    int keyframe_search_num = 20;
    bool loop_closure_fitness_use_adaptability = false;
    float loop_closure_fitness_score_thld_min = 0.05;
    float loop_closure_fitness_score_thld_max = 0.05;
    float icp_downsamp_size = 0.1;
    float loop_closure_height_thld = 20;

    pcl::PointCloud<PointXYZIRPYT>::Ptr copy_keyframe_pose6d;
    pcl::KdTreeFLANN<PointXYZIRPYT>::Ptr kdtree_history_keyframe_pose;

    unordered_map<int, int> loop_constraint_records; // <new, old>, keyframe index that has added loop constraint
    LoopConstraint loop_constraint;
    std::shared_ptr<ScanContext::SCManager> sc_manager; // scan context

    // for visualize
    double last_loop_time = 0;
    double dartion_time;
    PointCloudType::Ptr unused_result;
    PointCloudType::Ptr prevKeyframeCloud;
#ifdef USE_PANGOLIN
    ManuallyCorrectLoopClosure mclc;
#endif
};
