#pragma once
#include <fstream>
#include <Eigen/Eigen>
#include <vector>
#include <deque>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/geometry/Pose3.h>
#include "ikd-Tree/ikd_Tree.h"

#include "utility/LogTool.h"
#include "utility/Timer.h"
#include "utility/FileOperation.h"
#include "utility/EigenMath.h"
#include "utility/MathTools.h"

using namespace std;
using namespace Eigen;
using namespace EigenMath;

#define G_m_s2 (9.81) // Gravaty const in GuangDong/China

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define LEFT_MULTIPLY_QUA(v) -v[1], -v[2], -v[3], \
                             v[0], -v[3], v[2],   \
                             v[3], v[0], -v[1],   \
                             -v[2], v[1], v[0];
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "PCD/" + name))

/**
 * 6D位姿点云结构定义
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
    (double, time, time))

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using PointVector = KD_TREE<PointType>::PointVector;

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using V3F = Eigen::Vector3f;
using M3F = Eigen::Matrix3f;
using QD = Eigen::Quaterniond;
using QF = Eigen::Quaternionf;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

#define EYE3D (M3D::Identity())
#define EYE3F (M3F::Identity())
#define ZERO3D (V3D::Zero())
#define ZERO3F (V3F::Zero())
#define EYEQD (QD::Identity())
#define EYEQF (QF::Identity())

template <typename PointType>
float pointDistanceSquare(const PointType &p)
{
    return (p.x) * (p.x) + (p.y) * (p.y) + (p.z) * (p.z);
}

template <typename PointType>
float pointDistanceSquare(const PointType &p1, const PointType &p2)
{
    return pcl::squaredEuclideanDistance(p1, p2);
}

template <typename PointType>
float pointDistance(const PointType &p)
{
    return sqrt(pointDistanceSquare(p));
}

template <typename PointType>
float pointDistance(const PointType &p1, const PointType &p2)
{
    return sqrt(pointDistanceSquare(p1, p2));
}

inline gtsam::Pose3 pclPointTogtsamPose3(const PointXYZIRPYT &thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(thisPoint.roll, thisPoint.pitch, thisPoint.yaw),
                        gtsam::Point3(thisPoint.x, thisPoint.y, thisPoint.z));
}

inline Eigen::Affine3f pclPointToAffine3f(const PointXYZIRPYT &thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

inline void octreeDownsampling(const PointCloudType::Ptr &src, PointCloudType::Ptr &map_ds, const double &save_resolution)
{
    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(save_resolution);
    octree.setInputCloud(src);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    octree.getVoxelCentroids(centroids);

    map_ds->points.assign(centroids.begin(), centroids.end());
    map_ds->width = 1;
    map_ds->height = map_ds->points.size();
}

inline void savePCDFile(const std::string &save_path, const PointCloudType &src)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr save_pc(new pcl::PointCloud<pcl::PointXYZI>(src.points.size(), 1));
    for (auto i = 0; i < src.points.size(); ++i)
    {
        pcl::copyPoint(src.points[i], save_pc->points[i]);
    }
    pcl::io::savePCDFileBinary(save_path, *save_pc);
}

inline const bool compare_timestamp(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

inline bool check_for_not_converged(const double &timestamp, int step)
{
    static unsigned int cnt = 0;
    static double last_timestamp = 0;
    // LOG_WARN("check_for_not_converged = %f, %f, %f, %d", timestamp, last_timestamp, timestamp - last_timestamp, cnt);

    if (timestamp <= last_timestamp) // for test
        cnt = 0;

    if (cnt == 0)
    {
        last_timestamp = timestamp;
        ++cnt;
        return false;
    }

    bool flag = false;
    if (timestamp - last_timestamp > 60) // check only 60s
        return flag;

    if (cnt % step == 0)
    {
        if (timestamp - last_timestamp <= 1)
        {
            flag = true;
            // LOG_WARN("check_for_not_converged = %f, %d", timestamp - last_timestamp, cnt);
        }
        last_timestamp = timestamp;
    }
    ++cnt;
    return flag;
}

// #define DEDUB_MODE
