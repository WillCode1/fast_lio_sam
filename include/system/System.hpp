#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include <pcl/io/pcd_io.h>
#include "ikd-Tree/ikd_Tree.h"
#include "frontend/FastlioOdometry.hpp"
#include "frontend/PointlioOdometry.hpp"
#include "FactorGraphOptimization.hpp"
#include "LoopClosure.hpp"
#include "Relocalization.hpp"
#include "utility/Header.h"

class System
{
public:
    System()
    {
        save_resolution = 0.1;

        gnss = make_shared<GnssProcessor>();

        keyframe_pose6d_unoptimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
        keyframe_pose6d_optimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
        keyframe_scan.reset(new deque<PointCloudType::Ptr>());

        backend = std::make_shared<FactorGraphOptimization>(keyframe_pose6d_optimized, keyframe_scan, gnss);
        relocalization = make_shared<Relocalization>();
        loopClosure = make_shared<LoopClosure>(relocalization->sc_manager);

        feats_undistort.reset(new PointCloudType());

        file_pose_unoptimized = fopen(DEBUG_FILE_DIR("keyframe_pose.txt").c_str(), "w");
        file_pose_optimized = fopen(DEBUG_FILE_DIR("keyframe_pose_optimized.txt").c_str(), "w");

        fprintf(file_pose_unoptimized, "# keyframe trajectory unoptimized\n# timestamp tx ty tz qx qy qz qw\n");
        fprintf(file_pose_optimized, "# keyframe trajectory optimized\n# timestamp tx ty tz qx qy qz qw\n");
    }

    ~System()
    {
        if (loopthread.joinable())
            loopthread.join();

        fclose(file_pose_unoptimized);
        fclose(file_pose_optimized);
    }

    void init_system_mode(bool _map_update_mode)
    {
        map_update_mode = _map_update_mode;
        frontend->detect_range = frontend->lidar->detect_range;
        gnss->need_record_gnss = frontend->loger.runtime_log;
        frontend->init_estimator();
        loopthread = std::thread(&System::loopClosureThread, this);

        if (!map_update_mode)
        {
            FileOperation::createDirectoryOrRecreate(keyframe_path);
            FileOperation::createDirectoryOrRecreate(scd_path);
            return;
        }

        /*** init localization mode ***/
        save_keyframe_en = false;
        loop_closure_enable_flag = false;

        if (access(globalmap_path.c_str(), F_OK) != 0)
        {
            LOG_ERROR("File not exist! Please check the \"globalmap_path\".");
            std::exit(100);
        }

        PointCloudType::Ptr global_map(new PointCloudType());
        Timer timer;
        pcl::io::loadPCDFile(globalmap_path, *global_map);
        if (global_map->points.size() < 5000)
        {
            LOG_ERROR("Too few point clouds! Please check the map file.");
            std::exit(100);
        }
        LOG_WARN("Load pcd successfully! There are %lu points in map. Cost time %fms.", global_map->points.size(), timer.elapsedLast());

        if (!relocalization->load_prior_map(global_map))
        {
            std::exit(100);
        }

        pcl::io::loadPCDFile(trajectory_path, *relocalization->trajectory_poses);
        if (relocalization->trajectory_poses->points.size() < 10)
        {
            LOG_ERROR("Too few point clouds! Please check the trajectory file.");
            std::exit(100);
        }
        LOG_WARN("Load trajectory poses successfully! There are %lu poses.", relocalization->trajectory_poses->points.size());

        if (!relocalization->load_keyframe_descriptor(scd_path))
        {
            LOG_ERROR("Load keyframe descriptor failed!");
            std::exit(100);
        }
        LOG_WARN("Load keyframe descriptor successfully! There are %lu descriptors.", relocalization->sc_manager->polarcontexts_.size());

        /*** initialize the map kdtree ***/
        frontend->init_global_map(global_map);
    }

    bool run()
    {
        /*** relocalization for localization mode ***/
        if (map_update_mode && !system_state_vaild)
        {
            Eigen::Matrix4d imu_pose;
            if (relocalization->run(frontend->measures->lidar, imu_pose))
            {
                frontend->reset_state(imu_pose);
                system_state_vaild = true;
            }
            else
            {
                system_state_vaild = false;
                return system_state_vaild;
            }
        }

        /*** frontend ***/
        if (!frontend->run(map_update_mode, feats_undistort))
        {
            system_state_vaild = false;
            return system_state_vaild;
        }

        system_state_vaild = true;

        /*** backend ***/
        auto cur_state = frontend->get_state();
        backend->set_current_pose(frontend->lidar_end_time, cur_state, keyframe_pose6d_unoptimized->size());
        if (backend->is_keykrame())
        {
            // save keyframe info
            keyframe_pose6d_unoptimized->push_back(backend->this_pose6d);

            PointCloudType::Ptr this_keyframe(new PointCloudType());
            pcl::copyPointCloud(*feats_undistort, *this_keyframe);
            keyframe_scan->push_back(this_keyframe);
            relocalization->add_scancontext_descriptor(this_keyframe, scd_path);

            if (save_keyframe_en)
                save_keyframe(keyframe_scan->size());

            /*** loop closure ***/
            if (loop_closure_enable_flag && test_mode)
            {
                backend->get_keyframe_pose6d(loopClosure->copy_keyframe_pose6d);
                loopClosure->run(frontend->lidar_end_time, *keyframe_scan);
            }

            loopClosure->get_loop_constraint(loop_constraint);
#ifdef Ground_Constraint
            backend->add_ground_constraint = frontend->add_ground_constraint;
            backend->keyframe_rot = frontend->lidar_rot_meas;
#endif
            backend->run(loop_constraint, cur_state, frontend->ikdtree);
            frontend->set_pose(cur_state);
            return system_state_vaild;
        }

        return system_state_vaild;
    }

    void save_globalmap()
    {
        auto keyframe_num = keyframe_scan->size();
        PointCloudType::Ptr pcl_map_full(new PointCloudType());
        if (keyframe_pose6d_optimized->size() == keyframe_num)
            for (auto i = 0; i < keyframe_num; ++i)
                *pcl_map_full += *pointcloudKeyframeToWorld((*keyframe_scan)[i], (*keyframe_pose6d_optimized)[i]);
        else if (keyframe_pose6d_unoptimized->size() == keyframe_num)
            for (auto i = 0; i < keyframe_num; ++i)
                *pcl_map_full += *pointcloudKeyframeToWorld((*keyframe_scan)[i], (*keyframe_pose6d_unoptimized)[i]);
        else
            LOG_ERROR("no keyframe_num matched, when save global map!");

        pcl::VoxelGrid<pcl::PointXYZINormal> filter;
        filter.setLeafSize(save_resolution, save_resolution, save_resolution);
        filter.setInputCloud(pcl_map_full);
        filter.filter(*pcl_map_full);

        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(globalmap_path, *pcl_map_full);
        LOG_WARN("Success save global map to %s.", globalmap_path.c_str());
    }

    void save_trajectory()
    {
        int pose_num = keyframe_pose6d_unoptimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_unoptimized->points[i];
            const auto &state_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &state_pos = V3D(pose.x, pose.y, pose.z);
            LogAnalysis::save_trajectory(file_pose_unoptimized, state_pos, state_rot, pose.time);
        }
        LOG_WARN("Success save global unoptimized poses to file ...");

        pose_num = keyframe_pose6d_optimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_optimized->points[i];
            const auto &state_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &state_pos = V3D(pose.x, pose.y, pose.z);
            LogAnalysis::save_trajectory(file_pose_optimized, state_pos, state_rot, pose.time);
        }
        LOG_WARN("Success save global optimized poses to file ...");

        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(trajectory_path, *keyframe_pose6d_optimized);
        LOG_WARN("Success save trajectory poses to %s.", trajectory_path.c_str());
    }

    PointCloudType::Ptr get_submap_visual(float globalMapVisualizationSearchRadius, float globalMapVisualizationPoseDensity, float globalMapVisualizationLeafSize)
    {
        pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose(new pcl::PointCloud<PointXYZIRPYT>());
        backend->pose_mtx.lock();
        if (loop_closure_enable_flag)
            *keyframe_pose = *keyframe_pose6d_optimized;
        else
            *keyframe_pose = *keyframe_pose6d_unoptimized;
        backend->pose_mtx.unlock();

        if (keyframe_pose->points.empty())
            return PointCloudType::Ptr(nullptr);

        pcl::KdTreeFLANN<PointXYZIRPYT>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointXYZIRPYT>());
        pcl::PointCloud<PointXYZIRPYT>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointXYZIRPYT>());
        pcl::PointCloud<PointXYZIRPYT>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointXYZIRPYT>());
        PointCloudType::Ptr globalMapKeyFrames(new PointCloudType());
        PointCloudType::Ptr globalMapKeyFramesDS(new PointCloudType());

        // search near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        kdtreeGlobalMap->setInputCloud(keyframe_pose);
        kdtreeGlobalMap->radiusSearch(keyframe_pose->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(keyframe_pose->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames pose
        pcl::VoxelGrid<PointXYZIRPYT> downSizeFilterGlobalMapKeyPoses;
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for (auto &pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = keyframe_pose->points[pointSearchIndGlobalMap[0]].intensity;
        }

        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(globalMapKeyPosesDS->points[i], keyframe_pose->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *pointcloudKeyframeToWorld((*keyframe_scan)[thisKeyInd], keyframe_pose->points[thisKeyInd]);
        }
        // downsample key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        return globalMapKeyFramesDS;
    }

private:
    void save_keyframe(int keyframe_cnt, int num_digits = 6)
    {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt - 1;
        std::string keyframe_idx = out.str();
        string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << keyframe_file << endl;
        pcd_writer.writeBinary(keyframe_file, *feats_undistort);
    }

    void loopClosureThread()
    {
        if (loop_closure_enable_flag == false)
            return;

        LOG_WARN("loop closure enabled!");
        while (test_mode == false)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(loop_closure_interval));
            backend->get_keyframe_pose6d(loopClosure->copy_keyframe_pose6d);
            loopClosure->run(frontend->lidar_end_time, *keyframe_scan);
        }
    }

public:
    bool system_state_vaild = false; // true: system ok
    bool map_update_mode = false;  // true: localization, false: slam
    bool loop_closure_enable_flag = false;

    /*** sensor data processor ***/
    shared_ptr<GnssProcessor> gnss;

    /*** module ***/
    shared_ptr<FastlioOdometry> frontend;
    shared_ptr<FactorGraphOptimization> backend;
    shared_ptr<LoopClosure> loopClosure;
    shared_ptr<Relocalization> relocalization;

    int loop_closure_interval = 300;
    std::thread loopthread;
    LoopConstraint loop_constraint;
    bool test_mode = false;

    /*** keyframe config ***/
    FILE *file_pose_unoptimized;
    FILE *file_pose_optimized;
    bool save_keyframe_en = false;
    PointCloudType::Ptr feats_undistort;
    shared_ptr<deque<PointCloudType::Ptr>> keyframe_scan;

    /*** trajectory by lidar pose in camera_init frame(imu pose + extrinsic) ***/
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_unoptimized;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_optimized;

    /*** global map maintain ***/
    float save_resolution;
    string globalmap_path = PCD_FILE_DIR("globalmap.pcd");
    string trajectory_path = PCD_FILE_DIR("trajectory.pcd");
    string keyframe_path = PCD_FILE_DIR("keyframe/");
    string scd_path = PCD_FILE_DIR("scancontext/");
};
