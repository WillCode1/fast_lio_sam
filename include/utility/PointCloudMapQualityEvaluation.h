#pragma once
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <omp.h>
#include <vector>
using PointType = pcl::PointXYZ;
using namespace std;
/*
 *  pdf: Evaluation of Registration Methods for Sparse 3D Laser Scans
 *  pdf: Robust Odometry and Mapping for Multi-LiDAR Systems with Online Extrinsic Calibration
 *  Mean Map Entropy (MME) is proposed to measure the compactness of a point cloud.
 */
class PointCloudMapQualityEvaluation
{
public:
    PointCloudMapQualityEvaluation()
    {
        map_cloud.reset(new pcl::PointCloud<PointType>());
        kdtree.reset(new pcl::KdTreeFLANN<PointType>());
    }

    void LoadMap(const std::string &cloud_file_name)
    {
        pcl::io::loadPCDFile(cloud_file_name, *map_cloud);
    }

    void LoadMap(const pcl::PointCloud<PointType> &map)
    {
        *map_cloud = map;
    }

    double CalculateMeanMapEntropyMetrics(const double &searchRadius, int threads_num = 20)
    {
        kdtree->setInputCloud(map_cloud);
        double mme = 0;
        Eigen::Matrix3d covariance;
        Eigen::Vector4d centroid;
        double e = 2.718281828459045;

#pragma omp parallel for num_threads(threads_num)
        for (auto &point : map_cloud->points)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtree->radiusSearch(point, searchRadius, pointSearchInd, pointSearchSqDis, 0);

            if (pointSearchInd.size() < 2)
                continue;

            pcl::computeMeanAndCovarianceMatrix(*map_cloud, pointSearchInd, covariance, centroid);
            covariance *= 2 * M_PI * e;
            auto tmp = 0.5 * std::log(covariance.determinant()); // log = ln

            if (!isfinite(tmp))
            {
                continue;
            }

            mme += tmp;
        }

        mme /= map_cloud->points.size();
        return mme;
    }

private:
    pcl::PointCloud<PointType>::Ptr map_cloud;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree;
};
