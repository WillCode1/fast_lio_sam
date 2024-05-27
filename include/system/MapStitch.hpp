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

        PointCloudType::Ptr global_map(new PointCloudType());
        for (auto i = 1; i <= keyframe_pose6d_optimized->size(); ++i)
        {
            PointCloudType::Ptr keyframe_pc(new PointCloudType());
            load_keyframe(keyframe_path, keyframe_pc, i);
            octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
            keyframe_scan->push_back(keyframe_pc);
            *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d_optimized)[i - 1]);
        }
        octreeDownsampling(global_map, global_map, 0.3);
        if (!relocalization->load_prior_map(global_map))
        {
            std::exit(100);
        }

        load_factor_graph(path);

        for (auto i = 0; i < init_values.size(); ++i)
        {
            init_estimate.insert(i, init_values[i]);

            bool loop_is_closed = false;
            while (!gtsam_factors.empty() && gtsam_factors.front().index_to <= i)
            {
                gtsam::noiseModel::Diagonal::shared_ptr noise;
                auto &factor = gtsam_factors.front();
                if (factor.factor_type == GtsamFactor::Prior)
                {
                    noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << factor.noise).finished());
                    gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, factor.value, noise));
                }
                else if (factor.factor_type == GtsamFactor::Between || factor.factor_type == GtsamFactor::Loop)
                {
                    noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << factor.noise).finished());
                    gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(factor.index_from, factor.index_to, factor.value, noise));
                }
                else if (factor.factor_type == GtsamFactor::Gps)
                {
                    noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << factor.noise).finished());
                    gtsam_graph.add(gtsam::GPSFactor(factor.index_to, factor.value.translation(), noise));
                }

                if (factor.factor_type == GtsamFactor::Loop || factor.factor_type == GtsamFactor::Gps)
                {
                    loop_is_closed = true;
                }
                gtsam_factors.pop();
            }

            isam->update(gtsam_graph, init_estimate);
            isam->update();
            if (loop_is_closed == true)
            {
                isam->update();
                isam->update();
                isam->update();
                isam->update();
                isam->update();
            }
            gtsam_graph.resize(0);
            init_estimate.clear();
        }
    }

    void load_stitch_map_info(const std::string& path)
    {
        
    }

    void run()
    {

    }

    void load_keyframe(const std::string& keyframe_path, PointCloudType::Ptr keyframe_pc, int keyframe_cnt, int num_digits = 6)
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

    void load_factor_graph(const std::string& path)
    {
        FILE *ifs = fopen((path + "/factor_graph.fg").c_str(), "r");
        int value_size = 0;
        int factor_type = 0, index = 0, index2 = 0;
        double x, y, z, roll, pitch, yaw;
        double n1, n2, n3, n4, n5, n6;
        fscanf(ifs, "VERTEX_SIZE: %d\n", &value_size);
        for (auto i = 0; i < value_size; ++i)
        {
            fscanf(ifs, "VERTEX %d: %lf %lf %lf %lf %lf %lf\n", &index, &x, &y, &z, &roll, &pitch, &yaw);
            init_values[index] = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
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
            gtsam_factors.emplace(factor);
        }

        fclose(ifs);
        LOG_WARN("Success load factor graph, size = %ld.", isam->getFactorsUnsafe().size());
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

    std::map<int, gtsam::Pose3> init_values;
    std::queue<GtsamFactor> gtsam_factors;
};
