#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <thread>
#include <pcl/io/pcd_io.h>
#include "ikd-Tree/ikd_Tree.h"
#include "ImuProcessor.h"
#include "LidarProcessor.hpp"
#include "FastlioOdometry.hpp"
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

        lidar = make_shared<LidarProcessor>();
        imu = make_shared<ImuProcessor>();
        gnss = make_shared<GnssProcessor>();

        keyframe_pose6d_unoptimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
        keyframe_pose6d_optimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
        keyframe_scan.reset(new deque<PointCloudType::Ptr>());

        measures = make_shared<MeasureCollection>();
        frontend = make_shared<FastlioOdometry>();
        backend = std::make_shared<FactorGraphOptimization>(keyframe_pose6d_optimized, keyframe_scan, gnss);
        relocalization = make_shared<Relocalization>();
        loopClosure = make_shared<LoopClosure>(relocalization->sc_manager);

        feats_undistort.reset(new PointCloudType());

        file_pose_unoptimized = fopen(DEBUG_FILE_DIR("keyframe_pose.txt").c_str(), "w");
        file_pose_optimized = fopen(DEBUG_FILE_DIR("keyframe_pose_optimized.txt").c_str(), "w");
        file_pose_gnss = fopen(DEBUG_FILE_DIR("gnss_pose.txt").c_str(), "w");

        fprintf(file_pose_unoptimized, "# keyframe trajectory unoptimized\n# timestamp tx ty tz qx qy qz qw\n");
        fprintf(file_pose_optimized, "# keyframe trajectory optimized\n# timestamp tx ty tz qx qy qz qw\n");
        fprintf(file_pose_gnss, "# gnss trajectory\n# timestamp tx ty tz qx qy qz qw\n");
    }

    ~System()
    {
    }

    void init_system_mode(bool pure_localization)
    {
        localization_mode = pure_localization;
        frontend->detect_range = lidar->detect_range;

        double epsi[23] = {0.001};
        fill(epsi, epsi + 23, 0.001);
        auto lidar_meas_model = [&](state_ikfom &a, esekfom::dyn_share_datastruct<double> &b) { frontend->lidar_meas_model(a, b, loger); };
        frontend->kf.init_dyn_share(get_f, df_dx, df_dw, lidar_meas_model, frontend->num_max_iterations, epsi);

        if (!localization_mode)
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

    void cache_imu_data(double timestamp, const V3D &angular_velocity, const V3D &linear_acceleration)
    {
        timestamp = timestamp + timedelay_lidar2imu; // 时钟同步该采样同步td
        std::lock_guard<std::mutex> lock(mtx_buffer);

        if (timestamp < latest_timestamp_imu)
        {
            LOG_WARN("imu loop back, clear buffer");
            imu->imu_buffer.clear();
        }

        latest_timestamp_imu = timestamp;
        imu->imu_buffer.push_back(make_shared<ImuData>(latest_timestamp_imu, angular_velocity, linear_acceleration));
    }

    void cache_pointcloud_data(const double &lidar_beg_time, const PointCloudType::Ptr &scan)
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        if (lidar_beg_time < latest_lidar_beg_time)
        {
            LOG_ERROR("lidar loop back, clear buffer");
            lidar->lidar_buffer.clear();
        }

        latest_lidar_beg_time = lidar_beg_time;
        double latest_lidar_end_time = latest_lidar_beg_time + scan->points.back().curvature / 1000;

        if (abs(latest_lidar_end_time - latest_timestamp_imu) > 1)
        {
            LOG_WARN("IMU and LiDAR's clock not synced, IMU time: %lf, lidar time: %lf. Maybe set timedelay_lidar2imu = %lf.\n",
                     latest_timestamp_imu, latest_lidar_end_time, latest_lidar_end_time - latest_timestamp_imu);
        }

        lidar->lidar_buffer.push_back(scan);
        lidar->time_buffer.push_back(latest_lidar_beg_time);
    }

    bool run()
    {
        if (!sync_sensor_data())
            return false;

        if (loger.runtime_log && !loger.inited_first_lidar_beg_time)
        {
            loger.first_lidar_beg_time = measures->lidar_beg_time;
            loger.inited_first_lidar_beg_time = true;
        }

        /*** relocalization for localization mode ***/
        if (localization_mode && !system_state_vaild)
        {
            Eigen::Matrix4d imu_pose;
            if (relocalization->run(measures->lidar, imu_pose))
            {
                frontend->reset_state(imu_pose);
                system_state_vaild = true;
            }
            else
            {
#ifdef DEDUB_MODE
                frontend->reset_state(imu_pose);
#endif
                system_state_vaild = false;
                return system_state_vaild;
            }
        }

        /*** frontend ***/
        loger.resetTimer();
        if (!frontend->run(localization_mode, imu, *measures, feats_undistort, loger))
        {
            system_state_vaild = false;
            return system_state_vaild;
        }
        else if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            return false;
        }

        loger.update_average_time();
        loger.frame_log_output_to_csv(measures->lidar_beg_time);
#if 0
        // for test
        loger.save_trajectory(file_pose_unoptimized, frontend->state.pos, frontend->state.rot, measures->lidar_end_time);
#endif

        system_state_vaild = true;

        if (localization_mode)
            return system_state_vaild;

        /*** backend ***/
        backend->set_current_pose(measures->lidar_end_time, frontend->state, keyframe_pose6d_unoptimized->size());
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
            if (loop_closure_enable_flag)
            {
                backend->get_keyframe_pose6d(loopClosure->copy_keyframe_pose6d);
                if (!loopClosure->copy_keyframe_pose6d->empty())
                {
                    if (loopClosure->is_vaild_loop_time_period(loopClosure->copy_keyframe_pose6d->back().time - loopClosure->copy_keyframe_pose6d->front().time))
                    {
                        auto state_copy = frontend->state;
                        loopClosure->run(lidar_end_time, state_copy, *keyframe_scan);
                    }
                }
            }

            loopClosure->get_loop_constraint(loop_constraint);
            backend->run(loop_constraint, *frontend, loger);
            return system_state_vaild;
        }

        return system_state_vaild;
    }

    void save_globalmap()
    {
        if (localization_mode)
        {
            LOG_WARN("localization mode don't save map!");
            return;
        }

        auto keyframe_num = keyframe_scan->size();
        PointCloudType::Ptr pcl_map_full(new PointCloudType());
        if (keyframe_pose6d_optimized->size() == keyframe_num)
            for (auto i = 0; i < keyframe_num; ++i)
                *pcl_map_full += *pointcloudLidarToWorld((*keyframe_scan)[i], (*keyframe_pose6d_optimized)[i]);
        else if (keyframe_pose6d_unoptimized->size() == keyframe_num)
            for (auto i = 0; i < keyframe_num; ++i)
                *pcl_map_full += *pointcloudLidarToWorld((*keyframe_scan)[i], (*keyframe_pose6d_unoptimized)[i]);
        else
            LOG_ERROR("no keyframe_num matched, when save global map!");

        pcl::VoxelGrid<pcl::PointXYZINormal> filter;
        filter.setLeafSize(save_resolution, save_resolution, save_resolution);
        filter.setInputCloud(pcl_map_full);
        filter.filter(*pcl_map_full);

        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(globalmap_path, *pcl_map_full);
        LOG_WARN("Success save global map poses to %s.", globalmap_path.c_str());
    }

    void save_trajectory()
    {
        int pose_num = keyframe_pose6d_unoptimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_unoptimized->points[i];
            const auto &state_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &state_pos = V3D(pose.x, pose.y, pose.z);
            loger.save_trajectory(file_pose_unoptimized, state_pos, state_rot, pose.time);
        }
        LOG_WARN("Success save global unoptimized poses to file ...");

        pose_num = keyframe_pose6d_optimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_optimized->points[i];
            const auto &state_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &state_pos = V3D(pose.x, pose.y, pose.z);
            loger.save_trajectory(file_pose_optimized, state_pos, state_rot, pose.time);
        }
        LOG_WARN("Success save global optimized poses to file ...");

        if (!localization_mode)
        {
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(trajectory_path, *keyframe_pose6d_optimized);
            LOG_WARN("Success save trajectory poses to %s.", trajectory_path.c_str());
        }
    }

    PointCloudType::Ptr get_submap_visual(float globalMapVisualizationSearchRadius, float globalMapVisualizationPoseDensity, float globalMapVisualizationLeafSize)
    {
        if (localization_mode)
            return PointCloudType::Ptr(nullptr);

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
            *globalMapKeyFrames += *pointcloudLidarToWorld((*keyframe_scan)[thisKeyInd], keyframe_pose->points[thisKeyInd]);
        }
        // downsample key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        return globalMapKeyFramesDS;
    }

private:
    // 同步得到，当前帧激光点的开始和结束时间里的所有imu数据
    bool sync_sensor_data()
    {
        static bool lidar_pushed = false;
        static double lidar_mean_scantime = 0.0;
        static int scan_num = 0;

        std::lock_guard<std::mutex> lock(mtx_buffer);
        if (lidar->lidar_buffer.empty() || imu->imu_buffer.empty())
        {
            return false;
        }

        /*** push a lidar scan ***/
        if (!lidar_pushed)
        {
            measures->lidar = lidar->lidar_buffer.front();
            measures->lidar_beg_time = lidar->time_buffer.front();
            if (measures->lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = measures->lidar_beg_time + lidar_mean_scantime;
                LOG_WARN("Too few input point cloud!\n");
            }
            else if (measures->lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = measures->lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num++;
                lidar_end_time = measures->lidar_beg_time + measures->lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (measures->lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            measures->lidar_end_time = lidar_end_time;

            lidar_pushed = true;
        }

        if (latest_timestamp_imu < lidar_end_time)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = imu->imu_buffer.front()->timestamp;
        measures->imu.clear();
        while ((!imu->imu_buffer.empty()) && (imu_time <= lidar_end_time))
        {
            measures->imu.push_back(imu->imu_buffer.front());
#define RECORD_IMU_STATE
#ifdef RECORD_IMU_STATE
            angular_velocity = imu->imu_buffer.front()->angular_velocity;
            linear_acceleration = imu->imu_buffer.front()->linear_acceleration;
#endif
            imu->imu_buffer.pop_front();
            imu_time = imu->imu_buffer.front()->timestamp;
        }

        lidar->lidar_buffer.pop_front();
        lidar->time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

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

public:
    bool system_state_vaild = false; // true: system ok
    bool localization_mode = false;  // true: localization, false: slam
    bool loop_closure_enable_flag = false;
    LogAnalysis loger;

    /*** sensor data processor ***/
    shared_ptr<LidarProcessor> lidar;
    shared_ptr<ImuProcessor> imu;
    shared_ptr<GnssProcessor> gnss;

    double latest_lidar_beg_time = 0;
    double latest_timestamp_imu = -1.0;
    double timedelay_lidar2imu = 0.0;
    double lidar_end_time = 0;
    mutex mtx_buffer;

#ifdef RECORD_IMU_STATE
    V3D angular_velocity;
    V3D linear_acceleration;
#endif

    /*** module ***/
    shared_ptr<MeasureCollection> measures;
    shared_ptr<FastlioOdometry> frontend;
    shared_ptr<FactorGraphOptimization> backend;
    shared_ptr<LoopClosure> loopClosure;
    shared_ptr<Relocalization> relocalization;

    int loop_closure_interval = 2;
    LoopConstraint loop_constraint;

    /*** keyframe config ***/
    FILE *file_pose_unoptimized;
    FILE *file_pose_optimized;
    FILE *file_pose_gnss;
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
