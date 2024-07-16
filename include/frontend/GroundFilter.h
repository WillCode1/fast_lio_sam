#pragma once
#include <array>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "system/Header.h"


class GroundFilter
{
public:
    GroundFilter()
    {
        near_ground_points.reset(new PointCloudType);
        ground_points_inlier.reset(new PointCloudType);
        outer_points.reset(new PointCloudType);
    }

    void simple_ground_filter_by_lidar_height(const PointCloudType &pointcloud)
    {
        near_ground_points->clear();
        outer_points->clear();

        for (const auto &point : pointcloud)
        {
            if (std::sqrt(point.x * point.x + point.y * point.y) > distance)
                continue;

            if (std::abs(lidar_height + point.z) < error_thold)
                near_ground_points->push_back(point);
            else
                outer_points->push_back(point);
        }
    }

    // from GR-LOAM: https://sci-hub.se/https://doi.org/10.1016/j.robot.2021.103759
    void simple_ground_filter_by_theory_distance(const PointCloudType &pointcloud)
    {
        near_ground_points->clear();
        outer_points->clear();

        for (const auto &point : pointcloud)
        {
            if (std::sqrt(point.x * point.x + point.y * point.y) > distance)
                continue;

            auto cos_theta = std::abs(point.z / pointDistance(point));
            auto theory_dis = lidar_height / cos_theta;
            auto meas_dis = pointDistance(point);

            if (std::abs(meas_dis - theory_dis) < error_thold)
                near_ground_points->push_back(point);
            else
                outer_points->push_back(point);
        }
    }

    bool ground_seg_pcl()
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMaxIterations(200);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.03);
        seg.setInputCloud(near_ground_points);
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(near_ground_points);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground_points_inlier);

        ground_plane_normal = V3D(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        return true;
    }

    bool if_can_add_ground_constraint()
    {
        // 0.拟合地平面
        if (!ground_seg_pcl())
        {
            return false;
        }

        // 1.根据拟合的地平面计算分割的地面点云中内层的比例。如果该比例小于阈值，则认为地面不平整。
        if (1.0 * ground_points_inlier->size() / near_ground_points->size() < inlier_ratio)
        {
            return false;
        }

        // 2.计算地平面法线与机器人坐标系z轴之间的夹角。如果角度大于某个阈值，则认为地平面与运动平面不匹配。
        V3D robot_z(0, 0, 1);
        double cos_angle = ground_plane_normal.dot(robot_z) / (ground_plane_normal.norm() * robot_z.norm());
        double angle = RAD2DEG(std::acos(cos_angle));
        if (angle > normal_angle)
        {
            return false;
        }

        // 3.计算在分割的地面点集群中最远的地面点和最近的地面点相对于机器人的距离。如果距离最远点的距离低于阈值，距离最近点的距离高于阈值，则认为地面面积不连续。
        double nearest_distance = 1e6;
        double farthest_distance = 0;
        for (const auto &point : *near_ground_points)
        {
            auto distance = pointDistance(point);
            if (distance > farthest_distance)
                farthest_distance = distance;
            if (distance < nearest_distance)
                nearest_distance = distance;
        }
        if (farthest_distance < distant_threshold && nearest_distance > near_threshold)
        {
            return false;
        }

        return true;
    }

public:
    // double lidar_height = 2;
    double lidar_height = 2.183;
    double distance = 100;
    // double error_thold = 0.05;
    double error_thold = 0.1;
    // double error_thold = 0.2;

    double inlier_ratio = 0.9;
    double normal_angle = 3;
    double distant_threshold = 10;
    double near_threshold = 8;
    // double near_threshold = 2.7;
    Eigen::Vector3d ground_plane_normal;

    PointCloudType::Ptr near_ground_points;
    PointCloudType::Ptr ground_points_inlier;
    PointCloudType::Ptr outer_points;
};

// solid-state lidar
