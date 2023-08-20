#pragma once
#include <unordered_map>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "DataDef.h"
#include "use-ikfom.hpp"
#include "utility/Header.h"

std::vector<float> trans_state = std::vector<float>(6, 0);
std::vector<float> offset = std::vector<float>(6, 0.1);
std::vector<float> trans_state_backup;
int trans_state_index = 0;
int offset_index = 0;

double getFitnessScore(PointCloudType::Ptr submap, PointCloudType::Ptr scan, const Eigen::Affine3f &trans, double max_range)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudType::Ptr scan_tmp(new PointCloudType);
  pcl::transformPointCloud(*scan, *scan_tmp, trans);

  pcl::search::KdTree<PointType> kdtree;
  kdtree.setInputCloud(submap);

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < scan_tmp->points.size(); ++i)
  {
      // Find its nearest neighbor in the target
      kdtree.nearestKSearch(scan_tmp->points[i], 1, nn_indices, nn_dists);

      // Deal with occlusions (incomplete targets)
      if (nn_dists[0] <= max_range)
      {
          // Add to the fitness score
          fitness_score += nn_dists[0];
          nr++;
      }
  }

  if (nr > 0)
      return (fitness_score / nr);
  else
      return (std::numeric_limits<double>::max());
}

// 键盘事件回调函数
void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);

    if (event.getKeySym() == "q" && event.keyDown())
    {
        viewer->close();
    }
    else if (event.getKeySym() == "r" && event.keyDown())
    {
        trans_state = trans_state_backup;
    }
    else if (event.getKeySym() == "s" && event.keyDown())
    {
        viewer->saveScreenshot(string(ROOT_DIR) + "/screenshot.png");
    }
    else if (event.getKeySym() == "exclam" && event.keyDown())
    {
        offset_index = 0;
    }
    else if (event.getKeySym() == "at" && event.keyDown())
    {
        offset_index = 1;
    }
    else if (event.getKeySym() == "numbersign" && event.keyDown())
    {
        offset_index = 2;
    }
    else if (event.getKeySym() == "dollar" && event.keyDown())
    {
        offset_index = 3;
    }
    else if (event.getKeySym() == "percent" && event.keyDown())
    {
        offset_index = 4;
    }
    else if (event.getKeySym() == "asciicircum" && event.keyDown())
    {
        offset_index = 5;
    }
    else if (event.getKeySym() == "1")
    {
        trans_state_index = 0;
    }
    else if (event.getKeySym() == "2")
    {
        trans_state_index = 1;
    }
    else if (event.getKeySym() == "3")
    {
        trans_state_index = 2;
    }
    else if (event.getKeySym() == "4")
    {
        trans_state_index = 3;
    }
    else if (event.getKeySym() == "5")
    {
        trans_state_index = 4;
    }
    else if (event.getKeySym() == "6")
    {
        trans_state_index = 5;
    }
    else if (event.getKeySym() == "Up" && event.keyDown() && event.isShiftPressed())
    {
        offset[offset_index] += 0.01;
    }
    else if (event.getKeySym() == "Down" && event.keyDown() && event.isShiftPressed())
    {
        offset[offset_index] -= 0.01;
    }
    else if (event.getKeySym() == "Up" && event.keyDown())
    {
        trans_state[trans_state_index] += offset[trans_state_index];
    }
    else if (event.getKeySym() == "Down" && event.keyDown())
    {
        trans_state[trans_state_index] -= offset[trans_state_index];
    }
}

double manually_adjust_loop_closure(PointCloudType::Ptr submap, PointCloudType::Ptr scan, Eigen::Affine3f& transform)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // backup
    trans_state_backup = trans_state;

    transform = pcl::getTransformation(trans_state[0], trans_state[1], trans_state[2], trans_state[3], trans_state[4], trans_state[5]);
    PointCloudType::Ptr transformedCloud(new PointCloudType);
    pcl::transformPointCloud(*scan, *transformedCloud, transform);

    // 添加点云到可视化窗口
    viewer->addPointCloud<PointType>(submap, "submap");
    viewer->addPointCloud<PointType>(transformedCloud, "scan");

    // 设置点云渲染属性
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "submap");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "scan");

    char buffer[1024];
    sprintf(buffer, "current_pose (x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f), offset (x=%.2f, y=%.2f, z=%.2f, roll=%.3f, pitch=%.3f, yaw=%.3f)",
            trans_state[0], trans_state[1], trans_state[2], trans_state[3], trans_state[4], trans_state[5], offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]);
    viewer->addText(buffer, 50, 50, "text_1");

    // 注册键盘事件回调函数
    viewer->registerKeyboardCallback(keyboardEventCallback, viewer.get());

    double score = 0;
    // 循环显示
    while (!viewer->wasStopped())
    {
        score = getFitnessScore(submap, scan, transform, 2);
        sprintf(buffer, "current_pose (x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f), offset (x=%.2f, y=%.2f, z=%.2f, roll=%.3f, pitch=%.3f, yaw=%.3f), config=(%d, %d), FitnessScore=%lf",
                trans_state[0], trans_state[1], trans_state[2], trans_state[3], trans_state[4], trans_state[5],
                offset[0], offset[1], offset[2], offset[3], offset[4], offset[5], trans_state_index + 1, offset_index + 1, score);
        viewer->updateText(buffer, 50, 50, "text_1");

        transform = pcl::getTransformation(trans_state[0], trans_state[1], trans_state[2], trans_state[3], trans_state[4], trans_state[5]);
        pcl::transformPointCloud(*scan, *transformedCloud, transform);
        viewer->updatePointCloud<PointType>(transformedCloud, "scan");

        viewer->spinOnce();
    }

    viewer->close();
    return score;
}

class LoopClosure
{
public:
    LoopClosure()
    {
        copy_keyframe_pose6d.reset(new pcl::PointCloud<PointXYZIRPYT>());
        kdtree_history_keyframe_pose.reset(new pcl::KdTreeFLANN<PointXYZIRPYT>());
    }

    bool detect_loop_closure_by_distance(int &latest_id, int &closest_id, const double &lidar_end_time)
    {
        latest_id = copy_keyframe_pose6d->size() - 1; // 当前关键帧索引
        closest_id = -1;

        // 当前帧已经添加过闭环对应关系，不再继续添加
        auto it = loop_constraint_records.find(latest_id);
        if (it != loop_constraint_records.end())
            return false;
        // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree_history_keyframe_pose->setInputCloud(copy_keyframe_pose6d);
        kdtree_history_keyframe_pose->radiusSearch(copy_keyframe_pose6d->back(), loop_closure_search_radius, indices, distances, 0);
        // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
        for (int i = 0; i < (int)indices.size(); ++i)
        {
            int id = indices[i];
            if (abs(copy_keyframe_pose6d->points[id].time - lidar_end_time) > loop_closure_search_time_interval)
            {
                closest_id = id;
                break;
            }
        }
        if (closest_id == -1 || latest_id == closest_id)
            return false;
        return true;
    }

    /**
     * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
     */
    void loop_find_near_keyframes(PointCloudType::Ptr &near_keyframes, const int &key, const int &search_num,
                                  const state_ikfom &state, const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
        near_keyframes->clear();
        int cloudSize = copy_keyframe_pose6d->size();
        for (int i = -search_num; i <= search_num; ++i)
        {
            int key_near = key + i;
            if (key_near < 0 || key_near >= cloudSize)
                continue;

            *near_keyframes += *pointcloudLidarToWorld(keyframe_scan[key_near], copy_keyframe_pose6d->points[key_near]);
        }

        if (near_keyframes->empty())
            return;

        icp_downsamp_filter.setLeafSize(icp_downsamp_size, icp_downsamp_size, icp_downsamp_size);
        icp_downsamp_filter.setInputCloud(near_keyframes);
        icp_downsamp_filter.filter(*near_keyframes);
    }

    void run(const double lidar_end_time, const state_ikfom &state, const deque<PointCloudType::Ptr> &keyframe_scan)
    {
        if (copy_keyframe_pose6d->points.size() < loop_keyframe_num_thld)
        {
            return;
        }

        // 当前关键帧索引，候选闭环匹配帧索引
        int loop_key_cur;
        int loop_key_ref;
        // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
        if (detect_loop_closure_by_distance(loop_key_cur, loop_key_ref, lidar_end_time) == false)
        {
            return;
        }

        PointCloudType::Ptr cur_keyframe_cloud(new PointCloudType());
        PointCloudType::Ptr ref_near_keyframe_cloud(new PointCloudType());
        {
            // 提取当前关键帧特征点集合，降采样
            loop_find_near_keyframes(cur_keyframe_cloud, loop_key_cur, 0, state, keyframe_scan);
            // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
            loop_find_near_keyframes(ref_near_keyframe_cloud, loop_key_ref, keyframe_search_num, state, keyframe_scan);
            // 如果特征点较少，返回
            if (cur_keyframe_cloud->size() < 300 || ref_near_keyframe_cloud->size() < 1000)
            {
                return;
            }
            // 发布闭环匹配关键帧局部map
            // if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            //     publishCloud(&pubHistoryKeyFrames, ref_near_keyframe_cloud, timeLaserInfoStamp, odometryFrame);
        }

        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setMaxCorrespondenceDistance(loop_closure_search_radius * 2);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setRANSACIterations(0);

        gicp.setInputSource(cur_keyframe_cloud);
        gicp.setInputTarget(ref_near_keyframe_cloud);
        PointCloudType::Ptr unused_result(new PointCloudType());
        gicp.align(*unused_result);

        if (gicp.hasConverged() == false || gicp.getFitnessScore() > loop_closure_fitness_score_thld)
        {
            LOG_ERROR("loop closure failed, %d, %f, %f", gicp.hasConverged(), gicp.getFitnessScore(), loop_closure_fitness_score_thld);
            return;
        }

        // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
        // if (pubIcpKeyFrames.getNumSubscribers() != 0)
        // {
        //     PointCloudType::Ptr closed_cloud(new PointCloudType());
        //     pcl::pointcloudLidarToWorld(*cur_keyframe_cloud, *closed_cloud, gicp.getFinalTransformation());
        //     publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        // }

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = gicp.getFinalTransformation();
        float noiseScore = gicp.getFitnessScore();

        if (manually_fine_tune_loop_closure)
        {
            pcl::getTranslationAndEulerAngles(correctionLidarFrame, trans_state[0], trans_state[1], trans_state[2], trans_state[3], trans_state[4], trans_state[5]);
            noiseScore = manually_adjust_loop_closure(ref_near_keyframe_cloud, cur_keyframe_cloud, correctionLidarFrame);
        }

        // Get current frame wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_keyframe_pose6d->points[loop_key_cur]);
        // Get current frame corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        // Get reference frame pose
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_keyframe_pose6d->points[loop_key_ref]);
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

        loop_mtx.lock();
        loop_constraint.loop_indexs.push_back(make_pair(loop_key_cur, loop_key_ref));
        loop_constraint.loop_pose_correct.push_back(poseFrom.between(poseTo));
        loop_constraint.loop_noise.push_back(constraintNoise);
        loop_mtx.unlock();

        LOG_WARN("Loop Factor Added, noise = %f.", noiseScore);
        loop_constraint_records[loop_key_cur] = loop_key_ref;
    }

    void get_loop_constraint(LoopConstraint &loop_constr)
    {
        loop_mtx.lock();
        loop_constr = loop_constraint;
        loop_constraint.clear();
        loop_mtx.unlock();
    }

    bool is_vaild_loop_time_period(const double& time)
    {
        if (loop_closure_vaild_time_period.empty())
            return true;
        if (loop_closure_vaild_time_period.size() % 2 != 0)
        {
            LOG_ERROR("time_period size must be double!");
            return true;
        }

        for (auto i = 0; i < loop_closure_vaild_time_period.size(); i = i + 2)
        {
            if (loop_closure_vaild_time_period[i] > loop_closure_vaild_time_period[i + 1])
            {
                LOG_ERROR("time_period must before early than after!");
                continue;
            }
            if (time >= loop_closure_vaild_time_period[i] && time <= loop_closure_vaild_time_period[i + 1])
                return true;
        }

        return false;
    }

public:
    bool manually_fine_tune_loop_closure = false;
    std::vector<double> loop_closure_vaild_time_period;
    std::mutex loop_mtx;
    int loop_keyframe_num_thld = 50;
    float loop_closure_search_radius = 10;
    float loop_closure_search_time_interval = 30;
    int keyframe_search_num = 20;
    float loop_closure_fitness_score_thld = 0.05;
    float icp_downsamp_size = 0.1;
    pcl::VoxelGrid<PointType> icp_downsamp_filter;

    pcl::PointCloud<PointXYZIRPYT>::Ptr copy_keyframe_pose6d;
    pcl::KdTreeFLANN<PointXYZIRPYT>::Ptr kdtree_history_keyframe_pose;

    unordered_map<int, int> loop_constraint_records; // <new, old>, keyframe index that has added loop constraint
    LoopConstraint loop_constraint;
};
