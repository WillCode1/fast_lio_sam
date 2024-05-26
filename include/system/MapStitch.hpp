#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include "system/Header.h"
#include "FactorGraphOptimization.hpp"
#include "Relocalization.hpp"
#include "LoopClosure.hpp"

class MapStitch
{
public:
    MapStitch()
    {
        keyframe_pose6d_optimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
        keyframe_scan.reset(new deque<PointCloudType::Ptr>());

        relocalization = make_shared<Relocalization>();
        loopClosure = make_shared<LoopClosure>(relocalization->sc_manager);

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);
    }

    void load_prior_map_info(const std::string& path)
    {
        string globalmap_path = path + "/globalmap.pcd";
        string trajectory_path = path + "/trajectory.pcd";
        string keyframe_path = path + "/keyframe/";
        string scd_path = path + "/scancontext/";

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
        LOG_WARN("Success load trajectory poses %ld.", keyframe_pose6d_optimized->size());

        // load_factor_graph();
        // LOG_WARN("Success load factor graph, size = %ld.", backend->isam->getFactorsUnsafe().size());

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
    }

    void load_stitch_map_info(const std::string& path)
    {
        
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

public:
    shared_ptr<LoopClosure> loopClosure;
    shared_ptr<Relocalization> relocalization;

    shared_ptr<deque<PointCloudType::Ptr>> keyframe_scan;

    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_optimized;

    // gtsam
    gtsam::NonlinearFactorGraph gtsam_graph;
    gtsam::Values init_estimate;
    gtsam::Values optimized_estimate;
    gtsam::ISAM2 *isam;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_indoor;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_outdoor;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise;

    std::map<int, gtsam::Pose3> init_values;
    std::queue<GtsamFactor> gtsam_factors;
};
