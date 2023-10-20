#pragma once
#include <array>
#include "utility/Header.h"


class GroundFilter
{
public:
    GroundFilter()
    {
        ground_points.reset(new PointCloudType);
        outer_points.reset(new PointCloudType);
        last_ground_points.reset(new PointCloudType);
    }

    void simple_ground_filter_by_lidar_height(const PointCloudType &pointcloud, const double &lidar_height = 2, const double &error_thold = 0.05, const double &distance = 30)
    {
        *last_ground_points = *ground_points;
        ground_points->clear();
        outer_points->clear();

        for (const auto &point : pointcloud)
        {
            if (pointDistance(point) > distance)
                continue;
            if (std::abs(lidar_height + point.z) < error_thold)
                ground_points->push_back(point);
            else
                outer_points->push_back(point);
        }
    }

    // from GR-LOAM: https://sci-hub.se/https://doi.org/10.1016/j.robot.2021.103759
    void simple_ground_filter_by_theory_distance(const PointCloudType &pointcloud, const double &lidar_height = 2, const double &error_thold = 0.05)
    {
        *last_ground_points = *ground_points;
        ground_points->clear();
        outer_points->clear();

        // for rs-32-5515
        std::array<float, 32> ring2vertic_alangle = {15, 13, 11, 9, 7, 5.5, 4, 2.67, 1.33, 0, -1.33, -2.67, -4, -5.33, -6.67, -8,
                                                     -10, -16, -13, -19, -22, -28, -25, -31, -34, -37, -40, -43, -46, -49, -52, -55};

        for (const auto &point : pointcloud)
        {
            int ring = static_cast<int>(point.normal_x + 0.1);
            if (ring >= ring2vertic_alangle.size() || ring2vertic_alangle[ring] >= -1)
                continue;

            auto meas_dis = pointDistance(point);
            auto theory_dis = lidar_height / std::sin(-DEG2RAD(ring2vertic_alangle[ring]));
            if (std::abs(meas_dis - theory_dis) < error_thold)
                ground_points->push_back(point);
            else
                outer_points->push_back(point);
        }
    }

public:
    PointCloudType::Ptr ground_points;
    PointCloudType::Ptr outer_points;

    PointCloudType::Ptr last_ground_points;
};

// solid-state lidar
