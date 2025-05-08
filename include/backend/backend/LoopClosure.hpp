#pragma once
#include <unordered_map>
#include <condition_variable>
#include <atomic>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include "backend/Header.h"
#include "backend/utility/manually_correct_loop_closure.h"
#include "backend/global_localization/scancontext/Scancontext.h"

class LoopClosure
{
public:
    LoopClosure(const std::shared_ptr<ScanContext::SCManager> scManager, std::condition_variable &cv, std::atomic<bool> &isABlocked)
        : cv(cv), isABlocked(isABlocked)
    {
        copy_keyframe_pose6d.reset(new pcl::PointCloud<PointXYZIRPYT>());
        kdtree_history_keyframe_pose.reset(new pcl::KdTreeFLANN<PointXYZIRPYT>());

        unused_result.reset(new PointCloudType());
        prevKeyframeCloud.reset(new PointCloudType());

        loop_vaild_period["odom"] = std::vector<double>();
        loop_vaild_period["scancontext"] = std::vector<double>();
        sc_manager = scManager;
    }

    /**
     * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
     */
    void loop_find_near_keyframes(PointCloudType::Ptr &near_keyframes, const int &key, const int &search_num,
                                  const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
        near_keyframes->clear();
        int cloudSize = copy_keyframe_pose6d->size();
        for (int i = -search_num; i <= search_num; ++i)
        {
            int key_near = key + i;
            if (key_near < 0 || key_near >= cloudSize)
                continue;

            *near_keyframes += *pointcloudKeyframeToWorld(keyframe_scan[key_near], copy_keyframe_pose6d->points[key_near]);
        }

        if (near_keyframes->empty())
            return;

        octreeDownsampling(near_keyframes, near_keyframes, icp_downsamp_size);
    }

    void perform_loop_closure(const deque<PointCloudType::Ptr> &keyframe_scan, int loop_key_cur, int loop_key_ref,
                              const std::string &type, bool use_guess = false, const Eigen::Matrix4f &init_guess = Eigen::Matrix4f::Identity())
    {
        // extract cloud
        PointCloudType::Ptr cur_keyframe_cloud(new PointCloudType());
        PointCloudType::Ptr ref_near_keyframe_cloud(new PointCloudType());
        {
            loop_find_near_keyframes(cur_keyframe_cloud, loop_key_cur, 0, keyframe_scan);
            loop_find_near_keyframes(ref_near_keyframe_cloud, loop_key_ref, keyframe_search_num, keyframe_scan);
            if (cur_keyframe_cloud->size() < 300 || ref_near_keyframe_cloud->size() < 1000)
            {
                return;
            }

            // publish loop submap
            *prevKeyframeCloud = *ref_near_keyframe_cloud;
        }

        // GICP match
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setMaxCorrespondenceDistance(loop_closure_search_radius * 2);
        gicp.setMaximumIterations(1000);
        gicp.setTransformationEpsilon(1e-8);
        gicp.setEuclideanFitnessEpsilon(1e-8);
        gicp.setRANSACIterations(0);

        gicp.setInputSource(cur_keyframe_cloud);
        gicp.setInputTarget(ref_near_keyframe_cloud);
        if (use_guess)
            gicp.align(*unused_result, init_guess);
        else
            gicp.align(*unused_result);

        float loop_closure_fitness_score_thld = 0;
        if (loop_closure_fitness_use_adaptability)
        {
            if (dartion_time - last_loop_time > 40)
            {
                loop_closure_fitness_score_thld = loop_closure_fitness_score_thld_max;
            }
            else
            {
                loop_closure_fitness_score_thld = loop_closure_fitness_score_thld_min + (loop_closure_fitness_score_thld_max - loop_closure_fitness_score_thld_min) * 0.025 * (dartion_time - last_loop_time);
            }
        }
        else
            loop_closure_fitness_score_thld = loop_closure_fitness_score_thld_min;

        if (gicp.hasConverged() == false || gicp.getFitnessScore() > loop_closure_fitness_score_thld)
        {
            LOG_WARN("dartion_time = %.2f.loop closure failed by %s! %d, %.3f, %.3f", dartion_time, type.c_str(), gicp.hasConverged(), gicp.getFitnessScore(), loop_closure_fitness_score_thld);
            return;
        }

        bool reject_this_loop = false;
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame, posetransform, tuningLidarFrame;
        tuningLidarFrame.setIdentity();
        correctionLidarFrame = gicp.getFinalTransformation();
        float noiseScore = gicp.getFitnessScore();

#if 0
        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["manually"]))
        {
            // Get current frame wrong pose
            Eigen::Affine3f tWrong = pclPointToAffine3f(copy_keyframe_pose6d->points[loop_key_cur]);
            // Get current frame corrected pose
            Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;

            isABlocked.store(true);
            noiseScore = mclc.manually_adjust_loop_closure(ref_near_keyframe_cloud, keyframe_scan[loop_key_cur], copy_keyframe_pose6d, tCorrect, tuningLidarFrame, reject_this_loop);
            isABlocked.store(false);
            cv.notify_one();
        }
        if (reject_this_loop)
        {
            LOG_ERROR("dartion_time = %.2f. manually reject this loop closure! loop closure failed by %s!", dartion_time, type.c_str());
            return;
        }
#endif

        // Get current frame wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_keyframe_pose6d->points[loop_key_cur]);
        // Get current frame corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong * tuningLidarFrame;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        // Get reference frame pose
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_keyframe_pose6d->points[loop_key_ref]);
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

#if 1
        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["manually"]))
        {
            isABlocked.store(true);
            noiseScore = mclc.manually_adjust_loop_closure(ref_near_keyframe_cloud, keyframe_scan[loop_key_cur], copy_keyframe_pose6d, tCorrect, tuningLidarFrame, reject_this_loop);
            isABlocked.store(false);
            cv.notify_one();
        }
        if (reject_this_loop)
        {
            LOG_ERROR("dartion_time = %.2f. manually reject this loop closure! loop closure failed by %s!", dartion_time, type.c_str());
            return;
        }
#endif

        last_loop_time = dartion_time;

        loop_mtx.lock();
        loop_constraint.loop_indexs.push_back(make_pair(loop_key_cur, loop_key_ref));
        loop_constraint.loop_pose_correct.push_back(poseFrom.between(poseTo));
        loop_constraint.loop_noise.push_back(constraintNoise);
        loop_mtx.unlock();

        LOG_INFO("dartion_time = %.2f.Loop Factor Added by %s! keyframe id = %d, noise = %.3f.", dartion_time, type.c_str(), loop_key_ref, noiseScore);
        loop_constraint_records[loop_key_cur] = loop_key_ref;
    }

    void detect_loop_by_distance(const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        int latest_id = copy_keyframe_pose6d->size() - 1; // 当前关键帧索引
        int closest_id = -1;                              // 最近关键帧索引

        // 当前帧已经添加过闭环对应关系，不再继续添加
        auto it = loop_constraint_records.find(latest_id);
        if (it != loop_constraint_records.end())
            return;

        // 在历史关键帧中查找与当前关键帧距离最近的关键帧
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree_history_keyframe_pose->setInputCloud(copy_keyframe_pose6d);
        kdtree_history_keyframe_pose->radiusSearch(copy_keyframe_pose6d->back(), loop_closure_search_radius, indices, distances, 0);
        for (int i = 0; i < (int)indices.size(); ++i)
        {
            int id = indices[i];
            if (abs(id - latest_id) > loop_closure_keyframe_interval)
            {
                closest_id = id;
                break;
            }
        }
        if (closest_id == -1 || latest_id == closest_id)
            return;

        perform_loop_closure(keyframe_scan, latest_id, closest_id, "odom");
    }

    void detect_loop_by_scancontext(const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        int loop_key_cur = copy_keyframe_pose6d->size() - 1;

        auto detectResult = sc_manager->detectLoopClosureID(50); // first: nn index, second: yaw diff
        int loop_key_ref = detectResult.first;
        float sc_yaw_rad = detectResult.second; // sc2右移 <=> lidar左转 <=> 左+sc_yaw_rad

        if (loop_key_ref == -1)
            return;

        const auto &pose_ref = copy_keyframe_pose6d->points[loop_key_ref];
        Eigen::Matrix4f pose_ref_mat = EigenMath::CreateAffineMatrix(V3D(pose_ref.x, pose_ref.y, pose_ref.z), V3D(pose_ref.roll, pose_ref.pitch, pose_ref.yaw + sc_yaw_rad)).cast<float>();
        const auto &pose_cur = copy_keyframe_pose6d->back();
        Eigen::Matrix4f pose_cur_mat = EigenMath::CreateAffineMatrix(V3D(pose_cur.x, pose_cur.y, pose_cur.z), V3D(pose_cur.roll, pose_cur.pitch, pose_cur.yaw)).cast<float>();

        perform_loop_closure(keyframe_scan, loop_key_cur, loop_key_ref, "scancontext", true, pose_cur_mat.inverse() * pose_ref_mat);
    }

    void run(const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        if (copy_keyframe_pose6d->points.size() < loop_keyframe_num_thld)
        {
            return;
        }

        dartion_time = copy_keyframe_pose6d->back().time - copy_keyframe_pose6d->front().time;

        // 1.在历史关键帧中查找与当前关键帧距离最近的关键帧
        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["odom"]))
        {
            detect_loop_by_distance(keyframe_scan);
        }

        // 2.scan context
        if (is_vaild_loop_time_period(dartion_time, loop_vaild_period["scancontext"]))
        {
            detect_loop_by_scancontext(keyframe_scan);
        }
    }

    void get_loop_constraint(LoopConstraint &loop_constr)
    {
        loop_mtx.lock();
        loop_constr = loop_constraint;
        loop_constraint.clear();
        loop_mtx.unlock();
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
    ManuallyCorrectLoopClosure mclc;
};
