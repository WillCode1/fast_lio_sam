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
        keyframe_pose6d_prior.reset(new pcl::PointCloud<PointXYZIRPYT>());
        keyframe_pose6d_stitch.reset(new pcl::PointCloud<PointXYZIRPYT>());
        sc_manager_stitch = std::make_shared<ScanContext::SCManager>();

        relocalization = make_shared<Relocalization>();

        kdtree_history_keyframe_pose.reset(new pcl::KdTreeFLANN<PointXYZIRPYT>());

        loop_vaild_period["odom"] = std::vector<double>();
        loop_vaild_period["scancontext"] = std::vector<double>();

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);
    }

    void load_prior_map_info(const std::string &path)
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

        *keyframe_pose6d_prior = *relocalization->trajectory_poses;
        PointCloudType::Ptr global_map(new PointCloudType());
        for (auto i = 0; i < keyframe_pose6d_prior->size(); ++i)
        {
            PointCloudType::Ptr keyframe_pc(new PointCloudType());
            load_keyframe(keyframe_path, keyframe_pc, i);
            octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
            keyframe_scan_prior.push_back(keyframe_pc);
            *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d_prior)[i]);
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
            while (!gtsam_factors.empty() && gtsam_factors.top().index_to <= i)
            {
                gtsam::noiseModel::Diagonal::shared_ptr noise;
                auto &factor = gtsam_factors.top();
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
        init_values.clear();
        LOG_WARN("prior map finished!");
    }

    void load_stitch_map_info(const std::string &path)
    {
        string trajectory_path = path + "/trajectory.pcd";
        string keyframe_path = path + "/keyframe/";
        string scd_path = path + "/scancontext/";

        pcl::io::loadPCDFile(trajectory_path, *keyframe_pose6d_stitch);
        load_keyframe_descriptor(scd_path);

        for (auto i = 0; i < keyframe_pose6d_stitch->size(); ++i)
        {
            PointCloudType::Ptr keyframe_pc(new PointCloudType());
            load_keyframe(keyframe_path, keyframe_pc, i);
            octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
            keyframe_scan_stitch.push_back(keyframe_pc);
        }

        load_factor_graph(path, keyframe_pose6d_stitch->size());

        // 1.relocalization
        bool system_state_vaild = false;
        int first_index = 0;
        Eigen::Matrix4d imu_pose;
        for (first_index = 0; !system_state_vaild && first_index < keyframe_scan_stitch.size(); ++first_index)
        {
            // TODO: for gps
            system_state_vaild = relocalization->run(keyframe_scan_stitch[first_index], imu_pose, 100);
        }
        if (!system_state_vaild)
        {
            LOG_ERROR("all keyframe relocalization failed, end!");
            return;
        }
        first_index--;

        // 2.loop
        bool loop_add = false;
        relocalization->sc_manager->SC_DIST_THRES = 0.13;
        for (auto index = first_index; index < keyframe_scan_stitch.size(); ++index)
        {
            // fix pose to prior frame
            keyframe_pose6d_stitch->points[index];
            run_loop(index);

            if (!loop_constraint.loop_indexs.empty())
            {
                loop_add = true;
            }

        }

        // 4.factor graph optimize
        for (auto i = keyframe_pose6d_prior->size(); i < init_values.size() + keyframe_pose6d_prior->size(); ++i)
        {
            init_estimate.insert(i, pclPointTogtsamPose3(keyframe_pose6d_stitch->points[i - keyframe_pose6d_prior->size()]));

            bool loop_is_closed = false;
            while (!gtsam_factors.empty() && gtsam_factors.top().index_to <= i)
            {
                gtsam::noiseModel::Diagonal::shared_ptr noise;
                auto &factor = gtsam_factors.top();
                if (factor.factor_type == GtsamFactor::Between || factor.factor_type == GtsamFactor::Loop)
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

        init_values.clear();
        LOG_WARN("stitch map finished!");
    }

    void load_keyframe(const std::string &keyframe_path, PointCloudType::Ptr keyframe_pc, int keyframe_cnt, int num_digits = 6)
    {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt;
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

    bool load_keyframe_descriptor(const std::string &path)
    {
        if (!fs::exists(path))
        {
            LOG_WARN("path not exists, path = %s!", path.c_str());
            return false;
        }

        int scd_file_count = 0, num_digits = 0;
        scd_file_count = FileOperation::getFilesNumByExtension(path, ".scd");

        if (scd_file_count != keyframe_pose6d_stitch->size())
        {
            LOG_WARN("scd_file_count != trajectory_poses! %d, %ld", scd_file_count, keyframe_pose6d_stitch->size());
            return false;
        }

        num_digits = FileOperation::getOneFilenameByExtension(path, ".scd").length() - std::string(".scd").length();

        sc_manager_stitch->loadPriorSCD(path, num_digits, keyframe_pose6d_stitch->size());
        return true;
    }

    void load_factor_graph(const std::string &path, int index_offset = 0)
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
            init_values[index + index_offset] = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
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
                factor.index_from = index + index_offset;
                factor.index_to = index + index_offset;
                factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
                factor.noise.resize(6);
                factor.noise << std::pow(n1, 2), std::pow(n2, 2), std::pow(n3, 2), std::pow(n4, 2), std::pow(n5, 2), std::pow(n6, 2);
            }
            else if (factor_type == GtsamFactor::Between || factor_type == GtsamFactor::Loop)
            {
                fscanf(ifs, "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                       &index, &index2, &x, &y, &z, &roll, &pitch, &yaw, &n1, &n2, &n3, &n4, &n5, &n6);
                factor.factor_type = (GtsamFactor::FactorType)factor_type;
                factor.index_from = index + index_offset;
                factor.index_to = index2 + index_offset;
                factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
                factor.noise.resize(6);
                factor.noise << std::pow(n1, 2), std::pow(n2, 2), std::pow(n3, 2), std::pow(n4, 2), std::pow(n5, 2), std::pow(n6, 2);
            }
            else if (factor_type == GtsamFactor::Gps)
            {
                fscanf(ifs, "%d %lf %lf %lf %lf %lf %lf\n", &index, &x, &y, &z, &n1, &n2, &n3);
                factor.factor_type = (GtsamFactor::FactorType)factor_type;
                factor.index_from = index + index_offset;
                factor.index_to = index + index_offset;
                factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(x, y, z));
                factor.noise.resize(3);
                factor.noise << std::pow(n1, 2), std::pow(n2, 2), std::pow(n3, 2);
            }
            gtsam_factors.emplace(factor);
        }

        fclose(ifs);
        LOG_WARN("Success load factor graph, size = %ld.", gtsam_factors.size());
    }

private:
    /**
     * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
     */
    void loop_find_near_keyframes(PointCloudType::Ptr &near_keyframes, const int &key, const int &search_num,
                                  const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
        near_keyframes->clear();
        int cloudSize = keyframe_pose6d_prior->size();
        for (int i = -search_num; i <= search_num; ++i)
        {
            int key_near = key + i;
            if (key_near < 0 || key_near >= cloudSize)
                continue;

            *near_keyframes += *pointcloudKeyframeToWorld(keyframe_scan[key_near], keyframe_pose6d_prior->points[key_near]);
        }

        if (near_keyframes->empty())
            return;

        octreeDownsampling(near_keyframes, near_keyframes, icp_downsamp_size);
    }

    void perform_loop_closure(int loop_key_cur, int loop_key_ref,
                              const std::string &type, bool use_guess = false, const Eigen::Matrix4f &init_guess = Eigen::Matrix4f::Identity())
    {
        // extract cloud
        PointCloudType::Ptr cur_keyframe_cloud(new PointCloudType());
        PointCloudType::Ptr ref_near_keyframe_cloud(new PointCloudType());
        {
            *cur_keyframe_cloud = *pointcloudKeyframeToWorld(keyframe_scan_stitch[loop_key_cur], keyframe_pose6d_stitch->points[loop_key_cur]);
            loop_find_near_keyframes(ref_near_keyframe_cloud, loop_key_ref, keyframe_search_num, keyframe_scan_prior);
            if (cur_keyframe_cloud->size() < 300 || ref_near_keyframe_cloud->size() < 1000)
            {
                return;
            }
        }

        // GICP match
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setMaxCorrespondenceDistance(loop_closure_search_radius * 2);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setRANSACIterations(0);

        gicp.setInputSource(cur_keyframe_cloud);
        gicp.setInputTarget(ref_near_keyframe_cloud);
        PointCloudType::Ptr unused_result(new PointCloudType());
        if (use_guess)
            gicp.align(*unused_result, init_guess);
        else
            gicp.align(*unused_result);

        if (gicp.hasConverged() == false || gicp.getFitnessScore() > loop_closure_fitness_score_thld)
        {
            LOG_WARN("dartion_time = %.2f.loop closure failed by %s! %d, %.3f, %.3f", dartion_time, type.c_str(), gicp.hasConverged(), gicp.getFitnessScore(), loop_closure_fitness_score_thld);
            return;
        }

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = gicp.getFinalTransformation();
        float noiseScore = gicp.getFitnessScore();

        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["manually"]))
        {
            pcl::getTranslationAndEulerAngles(correctionLidarFrame, trans_state[0], trans_state[1], trans_state[2], trans_state[3], trans_state[4], trans_state[5]);
            noiseScore = manually_adjust_loop_closure(ref_near_keyframe_cloud, cur_keyframe_cloud, correctionLidarFrame);
        }

        // Get current frame wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(keyframe_pose6d_stitch->points[loop_key_cur]);
        // Get current frame corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        // Get reference frame pose
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(keyframe_pose6d_prior->points[loop_key_ref]);
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

        loop_constraint.loop_indexs.push_back(make_pair((int)keyframe_pose6d_prior->size() + loop_key_cur, loop_key_ref));
        loop_constraint.loop_pose_correct.push_back(poseFrom.between(poseTo));
        loop_constraint.loop_noise.push_back(constraintNoise);

        LOG_INFO("dartion_time = %.2f.Loop Factor Added by %s! keyframe id = %d, noise = %.3f.", dartion_time, type.c_str(), loop_key_ref, noiseScore);
    }

    void detect_loop_by_distance(int index)
    {
        int loop_key_cur = index; // 当前关键帧索引
        int loop_key_ref = -1;    // 最近关键帧索引

        // 在历史关键帧中查找与当前关键帧距离最近的关键帧
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree_history_keyframe_pose->setInputCloud(keyframe_pose6d_prior);
        kdtree_history_keyframe_pose->radiusSearch(keyframe_pose6d_stitch->points[index], loop_closure_search_radius, indices, distances, 0);

        if (indices.size() > 0)
            loop_key_ref = indices[0];
        else
            return;

        perform_loop_closure(loop_key_cur, loop_key_ref, "odom");
    }

    void detect_loop_by_scancontext(int index)
    {
        int loop_key_cur = index;

        auto detectResult = relocalization->sc_manager->detectClosestKeyframeID(0, sc_manager_stitch->polarcontext_invkeys_mat_[loop_key_cur], sc_manager_stitch->polarcontexts_[loop_key_cur]);
        // first: nn index, second: yaw diff
        int loop_key_ref = detectResult.first;
        float sc_yaw_rad = detectResult.second; // sc2右移 <=> lidar左转 <=> 左+sc_yaw_rad

        if (loop_key_ref == -1)
            return;

        const auto &pose_ref = keyframe_pose6d_prior->points[loop_key_ref];
        Eigen::Matrix4f pose_ref_mat = EigenMath::CreateAffineMatrix(V3D(pose_ref.x, pose_ref.y, pose_ref.z), V3D(pose_ref.roll, pose_ref.pitch, pose_ref.yaw + sc_yaw_rad)).cast<float>();
        const auto &pose_cur = keyframe_pose6d_stitch->points[loop_key_cur];
        Eigen::Matrix4f pose_cur_mat = EigenMath::CreateAffineMatrix(V3D(pose_cur.x, pose_cur.y, pose_cur.z), V3D(pose_cur.roll, pose_cur.pitch, pose_cur.yaw)).cast<float>();

        perform_loop_closure(loop_key_cur, loop_key_ref, "scancontext", true, pose_cur_mat.inverse() * pose_ref_mat);
    }

    void run_loop(int index)
    {
        dartion_time = keyframe_pose6d_stitch->points[index].time - keyframe_pose6d_stitch->front().time;

        // 1.在历史关键帧中查找与当前关键帧距离最近的关键帧
        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["odom"]))
        {
            detect_loop_by_distance(index);
        }

        // 2.scan context
        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["scancontext"]))
        {
            detect_loop_by_scancontext(index);
        }
    }

    bool is_vaild_loop_time_period(const double &time, const std::vector<double> &vaild_period)
    {
        if (vaild_period.empty())
            return true;
        if (vaild_period.size() % 2 != 0)
        {
            LOG_ERROR("time_period size must be double!");
            return true;
        }

        for (auto i = 0; i < vaild_period.size(); i = i + 2)
        {
            if (vaild_period[i] > vaild_period[i + 1])
            {
                LOG_ERROR("time_period must before early than after!");
                continue;
            }
            if (time >= vaild_period[i] && time <= vaild_period[i + 1])
                return true;
        }

        return false;
    }

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

    // gtsam
    gtsam::NonlinearFactorGraph gtsam_graph;
    gtsam::Values init_estimate;
    gtsam::Values optimized_estimate;
    gtsam::ISAM2 *isam;

    std::map<int, gtsam::Pose3> init_values;
    std::priority_queue<GtsamFactor> gtsam_factors;
};
