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

#include "DataDef.h"
#include "ikd-Tree/ikd_Tree.h"
#include "GnssProcessor.hpp"
#include "system/Header.h"

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

    bool operator<(const GtsamFactor &y) const
    {
        if (y.factor_type == Prior)
            return true;
        else if (factor_type == Prior)
            return false;
        else if (factor_type == Loop && y.factor_type == Loop)
            if (index_from == y.index_from)
                return index_to > y.index_to;
            else
                return index_from > y.index_from;

        auto value_x = std::max(index_from, index_to);
        auto value_y = std::max(y.index_from, y.index_to);
        if (value_x != value_y)
            return value_x > value_y;
        else
            return factor_type > y.factor_type;
    }

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
                            const shared_ptr<GnssProcessor> &p_gnss)
    {
        keyframe_pose6d_optimized = keyframe_pose;
        keyframe_scan = keyframe_cloud;
        gnss = p_gnss;

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);

        // rpy(rad*rad), xyz(meter*meter)
        prior_noise_indoor = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
        // prior_noise_outdoor = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
        prior_noise_outdoor = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1, 1, 1e-12).finished());
        odometry_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    }

    template <typename ikfom_state>
    void set_current_pose(const double &lidar_end_time, const ikfom_state &cur_state, uint32_t keyframe_index)
    {
        // imu pose -> lidar pose
        QD lidar_rot;
        V3D lidar_pos;
        poseTransformFrame(cur_state.rot, cur_state.pos, cur_state.offset_R_L_I, cur_state.offset_T_L_I, lidar_rot, lidar_pos);

        Eigen::Vector3d eulerAngle = EigenMath::Quaternion2RPY(lidar_rot);
        this_pose6d.x = lidar_pos(0); // x
        this_pose6d.y = lidar_pos(1); // y
        this_pose6d.z = lidar_pos(2); // z
        this_pose6d.intensity = keyframe_index;
        this_pose6d.roll = eulerAngle(0);  // roll
        this_pose6d.pitch = eulerAngle(1); // pitch
        this_pose6d.yaw = eulerAngle(2);   // yaw
        this_pose6d.time = lidar_end_time;
    }

    bool is_keyframe()
    {
        if (keyframe_pose6d_optimized->points.empty())
            return true;

        Eigen::Affine3f transLast = pclPointToAffine3f(keyframe_pose6d_optimized->back());
        Eigen::Affine3f transCurrent = pclPointToAffine3f(this_pose6d);
        Eigen::Affine3f transBetween = transLast.inverse() * transCurrent;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll) < keyframe_add_angle_threshold &&
            abs(pitch) < keyframe_add_angle_threshold &&
            abs(yaw) < keyframe_add_angle_threshold &&
            sqrt(x * x + y * y + z * z) < keyframe_add_dist_threshold)
            return false;
        return true;
    }

    template <typename ikfom_state>
    void run(LoopConstraint &loop_constraint, ikfom_state &state, KD_TREE<PointType> &ikdtree)
    {
        add_factor_and_optimize(loop_constraint, state);
        LOG_DEBUG("run backend 5");

        correct_poses(ikdtree);
    }

    void get_keyframe_pose6d(pcl::PointCloud<PointXYZIRPYT>::Ptr& copy_keyframe_pose6d)
    {
        pose_mtx.lock();
        *copy_keyframe_pose6d = *keyframe_pose6d_optimized;
        pose_mtx.unlock();
    }

private:
    void add_odom_factor()
    {
        if (keyframe_pose6d_optimized->points.empty())
        {
            gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pclPointTogtsamPose3(this_pose6d), prior_noise_outdoor));
            init_estimate.insert(0, pclPointTogtsamPose3(this_pose6d));

#ifdef MAP_STITCH
            GtsamFactor factor;
            factor.factor_type = GtsamFactor::Prior;
            factor.index_from = 0;
            factor.index_to = 0;
            factor.value = pclPointTogtsamPose3(this_pose6d);
            factor.noise = prior_noise_outdoor->covariance().diagonal();
            gtsam_factors.emplace(factor);
            init_values[0] = factor.value;
#endif
        }
        else
        {
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(keyframe_pose6d_optimized->points.back());
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(this_pose6d);
            gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(keyframe_pose6d_optimized->size() - 1, keyframe_pose6d_optimized->size(), poseFrom.between(poseTo), odometry_noise));
            init_estimate.insert(keyframe_pose6d_optimized->size(), poseTo);

#ifdef MAP_STITCH
            GtsamFactor factor;
            factor.factor_type = GtsamFactor::Between;
            factor.index_from = keyframe_pose6d_optimized->size() - 1;
            factor.index_to = keyframe_pose6d_optimized->size();
            factor.value = poseFrom.between(poseTo);
            factor.noise = odometry_noise->covariance().diagonal();
            gtsam_factors.emplace(factor);
            init_values[keyframe_pose6d_optimized->size()] = poseTo;
#endif
        }
    }

    void add_gnss_factor()
    {
        if (gnss->gnss_buffer.empty())
            return;
        if (keyframe_pose6d_optimized->points.empty())
            return;
        else if (pointDistance(keyframe_pose6d_optimized->front(), keyframe_pose6d_optimized->back()) < 5.0)
            return;
        if (std::hypot(pose_covariance(3, 3), pose_covariance(4, 4)) < pose_cov_threshold)
            return;
        // LOG_INFO("try to add GPS Factor!");
        GnssPose thisGPS;
        if (gnss->get_gnss_factor(thisGPS, this_pose6d.time, this_pose6d.z))
        {
            auto gnss_time_interval_weight = 1.0;
            gtsam::Vector Vector3(3);
            Vector3 << max(thisGPS.covariance(0), gnss_time_interval_weight), max(thisGPS.covariance(1), gnss_time_interval_weight), max(thisGPS.covariance(2), 0.05);
            gtsam::noiseModel::Diagonal::shared_ptr gnss_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(keyframe_pose6d_optimized->size(), gtsam::Point3(thisGPS.lidar_pos_fix(0), thisGPS.lidar_pos_fix(1), thisGPS.lidar_pos_fix(2)), gnss_noise);
            gtsam_graph.add(gps_factor);
            loop_is_closed = true;
            LOG_WARN("dartion_time = %.2f.GPS Factor Added, current_gnss_interval = %.3f sec, noise = (%.3f, %.3f, %.3f).",
                     thisGPS.timestamp - keyframe_pose6d_optimized->front().time, thisGPS.current_gnss_interval, Vector3(0), Vector3(1), Vector3(2));
            // LOG_INFO("fix_lidar_pos = (%.3f, %.3f, %.3f).", thisGPS.lidar_pos_fix(0), thisGPS.lidar_pos_fix(1), thisGPS.lidar_pos_fix(2));

#ifdef MAP_STITCH
            GtsamFactor factor;
            factor.factor_type = GtsamFactor::Gps;
            factor.index_from = keyframe_pose6d_optimized->size();
            factor.index_to = keyframe_pose6d_optimized->size();
            factor.value = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),
                                        gtsam::Point3(thisGPS.lidar_pos_fix(0), thisGPS.lidar_pos_fix(1), thisGPS.lidar_pos_fix(2)));
            factor.noise = gnss_noise->covariance().diagonal();
            gtsam_factors.emplace(factor);
#endif
        }
    }

    void add_loop_factor(LoopConstraint &loop_constraint)
    {
        if (loop_constraint.loop_indexs.empty())
            return;

        for (int i = 0; i < (int)loop_constraint.loop_indexs.size(); ++i)
        {
            int indexFrom = loop_constraint.loop_indexs[i].first; // cur
            int indexTo = loop_constraint.loop_indexs[i].second;  // pre
            const gtsam::Pose3& poseBetween = loop_constraint.loop_pose_correct[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loop_constraint.loop_noise[i];
            gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));

#ifdef MAP_STITCH
            GtsamFactor factor;
            factor.factor_type = GtsamFactor::Loop;
            factor.index_from = indexFrom;
            factor.index_to = indexTo;
            factor.value = poseBetween;
            factor.noise = noiseBetween->covariance().diagonal();
            gtsam_factors.emplace(factor);
#endif
        }

        loop_constraint.clear();
        loop_is_closed = true;
    }

    template <typename ikfom_state>
    void add_factor_and_optimize(LoopConstraint &loop_constraint, ikfom_state &state)
    {
        add_odom_factor();

        add_gnss_factor();

        add_loop_factor(loop_constraint);

        isam->update(gtsam_graph, init_estimate);
        isam->update();
        if (loop_is_closed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            LOG_INFO("ISMA2 Update");
        }
        // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
        gtsam_graph.resize(0);
        init_estimate.clear();

        optimized_estimate = isam->calculateBestEstimate();
        gtsam::Pose3 cur_estimate = optimized_estimate.at<gtsam::Pose3>(optimized_estimate.size() - 1);

        this_pose6d.x = cur_estimate.translation().x();
        this_pose6d.y = cur_estimate.translation().y();
        this_pose6d.z = cur_estimate.translation().z();
        // this_pose6d.intensity = keyframe_pose6d_optimized->size();
        this_pose6d.roll = cur_estimate.rotation().roll();
        this_pose6d.pitch = cur_estimate.rotation().pitch();
        this_pose6d.yaw = cur_estimate.rotation().yaw();
        // this_pose6d.time = lidar_end_time;
        pose_mtx.lock();
        keyframe_pose6d_optimized->push_back(this_pose6d);
        pose_mtx.unlock();

        pose_covariance = isam->marginalCovariance(optimized_estimate.size() - 1);

        if (loop_is_closed == true)
        {
            Eigen::Vector3d lidar_pos(this_pose6d.x, this_pose6d.y, this_pose6d.z);
            Eigen::Vector3d lidar_rot(this_pose6d.roll, this_pose6d.pitch, this_pose6d.yaw);
            poseTransformFrame2(EigenMath::RPY2Quaternion(lidar_rot), lidar_pos, state.offset_R_L_I, state.offset_T_L_I, state.rot, state.pos);
        }
    }

    void reset_ikdtree(KD_TREE<PointType> &ikdtree)
    {
        if (recontruct_kdtree)
        {
            PointCloudType::Ptr submap_keyframes(new PointCloudType());
            PointCloudType::Ptr submap_keyframesDS(new PointCloudType());

            int key_poses_num = keyframe_pose6d_optimized->size();
            for (int i = std::max(0, key_poses_num - ikdtree_reconstruct_keyframe_num); i < key_poses_num; ++i)
            {
                *submap_keyframes += *pointcloudKeyframeToWorld((*keyframe_scan)[i], keyframe_pose6d_optimized->points[i]);
            }
            octreeDownsampling(submap_keyframes, submap_keyframesDS, ikdtree_reconstruct_downsamp_size);

            ikdtree.reconstruct(submap_keyframesDS->points);
        }
    }

    void correct_poses(KD_TREE<PointType> &ikdtree)
    {
        if (keyframe_pose6d_optimized->points.empty())
            return;

        if (loop_is_closed == true)
        {
            int numPoses = optimized_estimate.size();
            pose_mtx.lock();
            for (int i = 0; i < numPoses; ++i)
            {
                keyframe_pose6d_optimized->points[i].x = optimized_estimate.at<gtsam::Pose3>(i).translation().x();
                keyframe_pose6d_optimized->points[i].y = optimized_estimate.at<gtsam::Pose3>(i).translation().y();
                keyframe_pose6d_optimized->points[i].z = optimized_estimate.at<gtsam::Pose3>(i).translation().z();
                keyframe_pose6d_optimized->points[i].roll = optimized_estimate.at<gtsam::Pose3>(i).rotation().roll();
                keyframe_pose6d_optimized->points[i].pitch = optimized_estimate.at<gtsam::Pose3>(i).rotation().pitch();
                keyframe_pose6d_optimized->points[i].yaw = optimized_estimate.at<gtsam::Pose3>(i).rotation().yaw();
#if 0
                LOG_WARN("optimized_pose = (%f, %f, %f, %f, %f, %f)",
                         keyframe_pose6d_optimized->points[i].x, keyframe_pose6d_optimized->points[i].y, keyframe_pose6d_optimized->points[i].z,
                         keyframe_pose6d_optimized->points[i].roll, keyframe_pose6d_optimized->points[i].pitch, keyframe_pose6d_optimized->points[i].yaw);
#endif
            }
            pose_mtx.unlock();
            reset_ikdtree(ikdtree);
            loop_is_closed = false;
        }
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::mutex pose_mtx;
    PointXYZIRPYT this_pose6d;
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d_optimized;
    shared_ptr<deque<PointCloudType::Ptr>> keyframe_scan;

    /* loop clousre */
    float pose_cov_threshold = 25;
    shared_ptr<GnssProcessor> gnss;
    bool loop_is_closed = false;

    // gtsam
    gtsam::NonlinearFactorGraph gtsam_graph;
    gtsam::Values init_estimate;
    gtsam::Values optimized_estimate;
    gtsam::ISAM2 *isam;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_indoor;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_outdoor;
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
};
