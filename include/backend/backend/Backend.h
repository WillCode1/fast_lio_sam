#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include <mutex>
#include "backend/Header.h"
#include "FactorGraphOptimization.h"
#include "Relocalization.h"
#include "LoopClosure.h"
#include "../utility/Pcd2Pgm.h"

class Backend
{
public:
    Backend();

    ~Backend();

    void init_system_mode();

    void run(PointXYZIRPYT &this_pose6d, PointCloudType::Ptr &feats_undistort, PointCloudType::Ptr &submap_fix);

    void save_globalmap();

    void save_pgm(const double &pgm_resolution = 0.05, const float &min_z = -1.5, const float &max_z = 0.1);

    void save_trajectory();

    // for ape
    void save_trajectory_to_other_frame(const QD &extR, const V3D &extP, const std::string& frame);

    void save_factor_graph();

    PointCloudType::Ptr get_submap_visual(float globalMapVisualizationSearchRadius, float globalMapVisualizationPoseDensity, float globalMapVisualizationLeafSize, bool showOptimizedPose = true);

    bool run_relocalization(PointCloudType::Ptr scan, const double &lidar_beg_time, Eigen::Matrix4d &imu_pose);

private:
    void load_keyframe(const std::string &keyframe_path, PointCloudType::Ptr keyframe_pc,
                       int keyframe_cnt, int num_digits = 6);

    void save_keyframe(PointCloudType::Ptr &cloud, int keyframe_cnt, int num_digits = 6);

    void loopClosureThread();

public:
    bool loop_closure_enable_flag = false;
    bool run_relocalization_thread = false;
    std::thread relocalization_thread;

    /*** sensor data processor ***/
    shared_ptr<GnssProcessor> gnss;

    /*** module ***/
    shared_ptr<FactorGraphOptimization> backend;
    shared_ptr<LoopClosure> loopClosure;
    shared_ptr<Relocalization> relocalization;

    int loop_closure_interval = 300;
    std::thread loopthread;
    LoopConstraint loop_constraint;
    bool test_mode = false;
    std::mutex m;
    std::condition_variable cv;
    std::atomic<bool> isABlocked;

    /*** keyframe config ***/
    bool save_keyframe_en = false;
    bool save_keyframe_descriptor_en = false;
    shared_ptr<deque<PointCloudType::Ptr>> keyframe_scan;

    /*** trajectory by lidar pose in camera_init frame(imu pose + extrinsic) ***/
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_unoptimized;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_optimized;

    /*** global map maintain ***/
    float save_resolution;
    string map_path;
    string globalmap_path = PCD_FILE_DIR("globalmap.pcd");
    string trajectory_path = PCD_FILE_DIR("trajectory.pcd");
    string keyframe_path = PCD_FILE_DIR("keyframe/");
    string scd_path = PCD_FILE_DIR("scancontext/");
};
