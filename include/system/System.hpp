#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include "ikd-Tree/ikd_Tree.h"
#include "frontend/FastlioOdometry.hpp"
#include "frontend/PointlioOdometry.hpp"
#include "FactorGraphOptimization.hpp"
#include "system/Header.h"
#include "Relocalization.hpp"
#include "LoopClosure.hpp"

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
    }

    ~System()
    {
        if (loopthread.joinable())
            loopthread.join();
    }

    void init_system_mode(bool _map_update_mode)
    {
        map_update_mode = _map_update_mode;
        frontend->detect_range = frontend->lidar->detect_range;
        frontend->init_estimator();
        loopthread = std::thread(&System::loopClosureThread, this);

        if (!map_update_mode)
        {
            FileOperation::createDirectoryOrRecreate(keyframe_path);
            FileOperation::createDirectoryOrRecreate(scd_path);
            return;
        }

        /*** init map update mode ***/
        loop_closure_enable_flag = false;

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

        pcl::PCDReader pcd_reader;
        pcd_reader.read(trajectory_path, *keyframe_pose6d_optimized);
        *keyframe_pose6d_unoptimized = *keyframe_pose6d_optimized;
        LOG_WARN("Success load trajectory poses %ld.", keyframe_pose6d_optimized->size());

        load_factor_graph();
        LOG_WARN("Success load factor graph, size = %ld.", backend->isam->getFactorsUnsafe().size());

        PointCloudType::Ptr global_map(new PointCloudType());
        for (auto i = 1; i <= keyframe_pose6d_optimized->size(); ++i)
        {
            PointCloudType::Ptr keyframe_pc(new PointCloudType());
            load_keyframe(keyframe_pc, i);
            octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
            keyframe_scan->push_back(keyframe_pc);
            *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d_optimized)[i - 1]);
        }
        octreeDownsampling(global_map, global_map, 0.3);
        if (!relocalization->load_prior_map(global_map))
        {
            std::exit(100);
        }

        /*** initialize the map kdtree ***/
        frontend->init_global_map(global_map);
        LOG_WARN("Success load last global map, point size = %ld.", global_map->size());
        loop_closure_enable_flag = true;
    }

    bool run()
    {
        /*** frontend ***/
        if (!frontend->run(map_update_mode, feats_undistort, gnss->extrinsic_lidar2gnss))
        {
            system_state_vaild = false;
            return system_state_vaild;
        }

        system_state_vaild = true;

        LOG_DEBUG("run backend 1");
        /*** backend ***/
        auto cur_state = frontend->get_state();
        backend->set_current_pose(frontend->lidar_end_time, cur_state, keyframe_pose6d_unoptimized->size());
        LOG_DEBUG("run backend 2");
        if (backend->is_keyframe())
        {
            // save keyframe info
            keyframe_pose6d_unoptimized->push_back(backend->this_pose6d);

            PointCloudType::Ptr this_keyframe(new PointCloudType());
            octreeDownsampling(feats_undistort, this_keyframe, 0.1);
            keyframe_scan->push_back(this_keyframe);

            if (save_keyframe_descriptor_en)
                relocalization->add_keyframe_descriptor(this_keyframe, scd_path);
            else
                relocalization->add_keyframe_descriptor(this_keyframe, "");

            if (save_keyframe_en)
                save_keyframe(keyframe_scan->size());

            /*** loop closure ***/
            if (loop_closure_enable_flag && test_mode)
            {
                backend->get_keyframe_pose6d(loopClosure->copy_keyframe_pose6d);
                loopClosure->run(frontend->lidar_end_time, *keyframe_scan);
            }

            LOG_DEBUG("run backend 3");
            loopClosure->get_loop_constraint(loop_constraint);
            LOG_DEBUG("run backend 4");
            backend->run(loop_constraint, cur_state, frontend->ikdtree);
            LOG_DEBUG("run backend 6");
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

        octreeDownsampling(pcl_map_full, pcl_map_full, save_resolution);
        savePCDFile(globalmap_path, *pcl_map_full);
        LOG_WARN("Success save global map to %s.", globalmap_path.c_str());
    }

    void save_trajectory()
    {
        FILE *file_pose_unoptimized = fopen(DEBUG_FILE_DIR("keyframe_pose.txt").c_str(), "w");
        fprintf(file_pose_unoptimized, "# keyframe trajectory unoptimized\n# timestamp tx ty tz qx qy qz qw\n");
        int pose_num = keyframe_pose6d_unoptimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_unoptimized->points[i];
            const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
            LogAnalysis::save_trajectory(file_pose_unoptimized, lidar_pos, lidar_rot, pose.time);
        }
        LOG_WARN("Success save global unoptimized lidar poses to file ...");
        fclose(file_pose_unoptimized);

        FILE *file_pose_optimized = fopen(DEBUG_FILE_DIR("keyframe_pose_optimized.txt").c_str(), "w");
        fprintf(file_pose_optimized, "# keyframe trajectory optimized\n# timestamp tx ty tz qx qy qz qw\n");
        pose_num = keyframe_pose6d_optimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_optimized->points[i];
            const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
            LogAnalysis::save_trajectory(file_pose_optimized, lidar_pos, lidar_rot, pose.time);
        }
        LOG_WARN("Success save global optimized lidar poses to file ...");
        fclose(file_pose_optimized);

        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(trajectory_path, *keyframe_pose6d_optimized);
        LOG_WARN("Success save trajectory poses to %s.", trajectory_path.c_str());

        if (map_path.compare("") != 0)
            fs::copy_file(DEBUG_FILE_DIR("keyframe_pose_optimized.txt"), map_path + "/keyframe_pose_optimized.txt", fs::copy_options::overwrite_existing);
    }

    // for ape
    void save_trajectory_to_other_frame(const QD &extR, const V3D &extP, const std::string& frame)
    {
        FILE *file_pose_unoptimized_imu = fopen(DEBUG_FILE_DIR("keyframe_pose_" + frame + ".txt").c_str(), "w");
        fprintf(file_pose_unoptimized_imu, "# keyframe trajectory unoptimized in %s frame\n# timestamp tx ty tz qx qy qz qw\n", frame.c_str());
        const auto &state = frontend->get_state();
        int pose_num = keyframe_pose6d_unoptimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_unoptimized->points[i];
            const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
            QD imu_rot;
            V3D imu_pos;
            poseTransformFrame2(lidar_rot, lidar_pos, extR, extP, imu_rot, imu_pos);
            LogAnalysis::save_trajectory(file_pose_unoptimized_imu, imu_pos, imu_rot, pose.time);
        }
        LOG_WARN("Success save global unoptimized %s poses to file ...", frame.c_str());
        fclose(file_pose_unoptimized_imu);

        FILE *file_pose_optimized_imu = fopen(DEBUG_FILE_DIR("keyframe_pose_optimized_" + frame + ".txt").c_str(), "w");
        fprintf(file_pose_optimized_imu, "# keyframe trajectory optimized in %s frame\n# timestamp tx ty tz qx qy qz qw\n", frame.c_str());
        pose_num = keyframe_pose6d_optimized->points.size();
        for (auto i = 0; i < pose_num; ++i)
        {
            const auto &pose = keyframe_pose6d_optimized->points[i];
            const auto &lidar_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
            const auto &lidar_pos = V3D(pose.x, pose.y, pose.z);
            QD imu_rot;
            V3D imu_pos;
            poseTransformFrame2(lidar_rot, lidar_pos, extR, extP, imu_rot, imu_pos);
            LogAnalysis::save_trajectory(file_pose_optimized_imu, imu_pos, imu_rot, pose.time);
        }
        LOG_WARN("Success save global optimized %s poses to file ...", frame.c_str());
        fclose(file_pose_optimized_imu);
    }

    void save_factor_graph()
    {
        if (map_path.compare("") == 0)
        {
            LOG_WARN("please set map_path!");
            return;
        }

        FILE *ofs = fopen((map_path + "/factor_graph.fg").c_str(), "w");
        fprintf(ofs, "VERTEX_SIZE: %ld\n", backend->init_values.size());
        for (auto &value : backend->init_values)
        {
            fprintf(ofs, "VERTEX %d: %lf %lf %lf %lf %lf %lf\n",
                    value.first, value.second.x(), value.second.y(), value.second.z(),
                    value.second.rotation().roll(), value.second.rotation().pitch(), value.second.rotation().yaw());
        }
        fprintf(ofs, "EDGE_SIZE: %ld\n", backend->gtsam_factors.size());
        while (!backend->gtsam_factors.empty())
        {
            auto &factor = backend->gtsam_factors.front();
            if (factor.factor_type == GtsamFactor::Prior)
            {
                fprintf(ofs, "EDGE %d: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                        factor.factor_type, factor.index_to, factor.value.x(), factor.value.y(), factor.value.z(),
                        factor.value.rotation().roll(), factor.value.rotation().pitch(), factor.value.rotation().yaw(),
                        std::sqrt(factor.noise(0)), std::sqrt(factor.noise(1)), std::sqrt(factor.noise(2)),
                        std::sqrt(factor.noise(3)), std::sqrt(factor.noise(4)), std::sqrt(factor.noise(5)));
            }
            else if (factor.factor_type == GtsamFactor::Between || factor.factor_type == GtsamFactor::Loop)
            {
                fprintf(ofs, "EDGE %d: %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                        factor.factor_type, factor.index_from, factor.index_to, 
                        factor.value.x(), factor.value.y(), factor.value.z(),
                        factor.value.rotation().roll(), factor.value.rotation().pitch(), factor.value.rotation().yaw(),
                        std::sqrt(factor.noise(0)), std::sqrt(factor.noise(1)), std::sqrt(factor.noise(2)),
                        std::sqrt(factor.noise(3)), std::sqrt(factor.noise(4)), std::sqrt(factor.noise(5)));
            }
            else if (factor.factor_type == GtsamFactor::Gps)
            {
                fprintf(ofs, "EDGE %d: %d %lf %lf %lf %lf %lf %lf\n",
                        factor.factor_type, factor.index_to, 
                        factor.value.x(), factor.value.y(), factor.value.z(),
                        std::sqrt(factor.noise(0)), std::sqrt(factor.noise(1)), std::sqrt(factor.noise(2)));
            }
            backend->gtsam_factors.pop();
        }

        fclose(ofs);
    }

    void load_factor_graph()
    {
        if (map_path.compare("") == 0)
        {
            LOG_WARN("please set map_path!");
            return;
        }

        FILE *ifs = fopen((map_path + "/factor_graph.fg").c_str(), "r");
        int value_size = 0;
        int factor_type = 0, index = 0, index2 = 0;
        double x, y, z, roll, pitch, yaw;
        double n1, n2, n3, n4, n5, n6;
        fscanf(ifs, "VERTEX_SIZE: %d\n", &value_size);
        for (auto i = 0; i < value_size; ++i)
        {
            fscanf(ifs, "VERTEX %d: %lf %lf %lf %lf %lf %lf\n", &index, &x, &y, &z, &roll, &pitch, &yaw);
            backend->init_values[index] = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        }
        fscanf(ifs, "EDGE_SIZE: %d\n", &value_size);
        for (auto i = 0; i < value_size; ++i)
        {
            fscanf(ifs, "EDGE %d: ", &factor_type);
            GtsamFactor factor;
            if (factor_type == GtsamFactor::Prior)
            {
                fscanf(ifs, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                       &index, &x, &y, &z, &roll, &pitch, &yaw, &n1, &n2, &n3, &n4, &n5, &n6);
                factor.factor_type = (GtsamFactor::FactorType)factor_type;
                factor.index_from = index;
                factor.index_to = index;
                factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
                factor.noise.resize(6);
                factor.noise << std::pow(n1, 2), std::pow(n2, 2), std::pow(n3, 2), std::pow(n4, 2), std::pow(n5, 2), std::pow(n6, 2);
            }
            else if (factor_type == GtsamFactor::Between || factor_type == GtsamFactor::Loop)
            {
                fscanf(ifs, "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                       &index, &index2, &x, &y, &z, &roll, &pitch, &yaw, &n1, &n2, &n3, &n4, &n5, &n6);
                factor.factor_type = (GtsamFactor::FactorType)factor_type;
                factor.index_from = index;
                factor.index_to = index2;
                factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
                factor.noise.resize(6);
                factor.noise << std::pow(n1, 2), std::pow(n2, 2), std::pow(n3, 2), std::pow(n4, 2), std::pow(n5, 2), std::pow(n6, 2);
            }
            else if (factor_type == GtsamFactor::Gps)
            {
                fscanf(ifs, "%d %lf %lf %lf %lf %lf %lf\n", &index, &x, &y, &z, &n1, &n2, &n3);
                factor.factor_type = (GtsamFactor::FactorType)factor_type;
                factor.index_from = index;
                factor.index_to = index;
                factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(x, y, z));
                factor.noise.resize(3);
                factor.noise << std::pow(n1, 2), std::pow(n2, 2), std::pow(n3, 2);
            }
            backend->gtsam_factors.emplace(factor);
        }

        fclose(ifs);
    }

    PointCloudType::Ptr get_submap_visual(float globalMapVisualizationSearchRadius, float globalMapVisualizationPoseDensity, float globalMapVisualizationLeafSize, bool showOptimizedPose = true)
    {
        pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose(new pcl::PointCloud<PointXYZIRPYT>());
        backend->pose_mtx.lock();
        if (showOptimizedPose)
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
        octreeDownsampling(globalMapKeyFrames, globalMapKeyFramesDS, globalMapVisualizationLeafSize);
        return globalMapKeyFramesDS;
    }

    bool run_relocalization(PointCloudType::Ptr scan, const double &lidar_beg_time)
    {
        run_relocalization_thread = true;
        if (map_update_mode && !system_state_vaild)
        {
            Eigen::Matrix4d imu_pose;
            if (relocalization->run(scan, imu_pose, lidar_beg_time))
            {
                frontend->reset_state(imu_pose);
                system_state_vaild = true;
            }
        }
        run_relocalization_thread = false;
        return system_state_vaild;
    }

private:
    void save_keyframe(int keyframe_cnt, int num_digits = 6)
    {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt - 1;
        std::string keyframe_idx = out.str();
        string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
        savePCDFile(keyframe_file, *feats_undistort);
    }

    void load_keyframe(PointCloudType::Ptr keyframe_pc, int keyframe_cnt, int num_digits = 6)
    {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt - 1;
        std::string keyframe_idx = out.str();
        string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(keyframe_file, *tmp_pc);
        keyframe_pc->points.resize(tmp_pc->points.size());
        for (auto i = 0; i < tmp_pc->points.size(); ++i)
        {
            pcl::copyPoint(tmp_pc->points[i], keyframe_pc->points[i]);
        }
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
    bool run_relocalization_thread = false;
    std::thread relocalization_thread;

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
    bool save_keyframe_en = false;
    bool save_keyframe_descriptor_en = false;
    PointCloudType::Ptr feats_undistort;
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
