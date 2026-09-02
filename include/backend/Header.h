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
#include <gtsam/linear/NoiseModel.h>

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

struct ImuState
{
    ImuState(const double &time = 0, const V3D &a = ZERO3D, const V3D &g = ZERO3D,
             const V3D &v = ZERO3D, const V3D &p = ZERO3D, const M3D &r = EYE3D)
        : offset_time(time), acc(a), gyr(g), vel(v), pos(p), rot(r) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset_time;
    V3D acc;
    V3D gyr;
    V3D vel;
    V3D pos;
    M3D rot;
};

struct ImuData
{
    using Ptr = std::shared_ptr<ImuData>;

    ImuData(const double &t = 0, const V3D &av = ZERO3D, const V3D &la = ZERO3D, const QD &ori = QD::Identity())
    {
        timestamp = t;
        angular_velocity = av;
        linear_acceleration = la;
        orientation = ori;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    V3D angular_velocity;
    V3D linear_acceleration;
    QD orientation;
};

struct MeasureCollection
{
    MeasureCollection()
    {
        lidar_beg_time = 0.0;
        lidar.reset(new PointCloudType());
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudType::Ptr lidar;
    deque<ImuData::Ptr> imu;
};

struct LoopConstraint
{
    void clear()
    {
        loop_indexs.clear();
        loop_pose_correct.clear();
        loop_noise.clear();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector<pair<int, int>> loop_indexs;
    vector<gtsam::Pose3> loop_pose_correct;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loop_noise;
};

/**
 * @brief transform frame_a to frame_b
 * @param extR rot from frame_b to frame_a
 * @param extP pos from frame_b to frame_a
 */
template <typename T>
void poseTransformFrame(const Eigen::Quaternion<T> &rot_from, const Eigen::Matrix<T, 3, 1> &pos_from,
                        const Eigen::Quaternion<T> &extR, const Eigen::Matrix<T, 3, 1> &extP,
                        Eigen::Quaternion<T> &rot_to, Eigen::Matrix<T, 3, 1> &pos_to)
{
    rot_to = (rot_from * extR).normalized();
    pos_to = rot_from.normalized() * extP + pos_from;
}

/**
 * @brief transform frame_a to frame_b
 * @param extR rot from frame_a to frame_b
 * @param extP pos from frame_a to frame_b
 */
template <typename T>
void poseTransformFrame2(const Eigen::Quaternion<T> &rot_from, const Eigen::Matrix<T, 3, 1> &pos_from,
                         const Eigen::Quaternion<T> &extR, const Eigen::Matrix<T, 3, 1> &extP,
                         Eigen::Quaternion<T> &rot_to, Eigen::Matrix<T, 3, 1> &pos_to)
{
    rot_to = (rot_from * extR.conjugate()).normalized();
    pos_to = pos_from - rot_to * extP;
}

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

template <typename ikfom_state>
void state2pose(PointXYZIRPYT &this_pose6d, const double &lidar_end_time, const ikfom_state &state)
{
    // imu pose -> lidar pose
    QD lidar_rot;
    V3D lidar_pos;
    poseTransformFrame(state.rot, state.pos, state.offset_R_L_I, state.offset_T_L_I, lidar_rot, lidar_pos);

    Eigen::Vector3d eulerAngle = EigenMath::Quaternion2RPY(lidar_rot);
    this_pose6d.x = lidar_pos(0); // x
    this_pose6d.y = lidar_pos(1); // y
    this_pose6d.z = lidar_pos(2); // z
    this_pose6d.roll = eulerAngle(0);  // roll
    this_pose6d.pitch = eulerAngle(1); // pitch
    this_pose6d.yaw = eulerAngle(2);   // yaw
    this_pose6d.time = lidar_end_time;
}

template <typename ikfom_state>
void pose2state(const PointXYZIRPYT &this_pose6d, ikfom_state &state)
{
    // lidar pose -> imu pose
    V3D lidar_pos = V3D(this_pose6d.x, this_pose6d.y, this_pose6d.z);
    V3D eulerAngle = V3D(this_pose6d.roll, this_pose6d.pitch, this_pose6d.yaw);
    QD lidar_rot = EigenMath::RPY2Quaternion(eulerAngle);
    poseTransformFrame2(lidar_rot, lidar_pos, state.offset_R_L_I, state.offset_T_L_I, state.rot, state.pos);
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

template <typename ikfom_state>
void pointLidarToWorld(V3D const &p_lidar, V3D &p_imu, const ikfom_state &state)
{
    p_imu = state.offset_R_L_I.normalized() * p_lidar + state.offset_T_L_I;
}

template <typename ikfom_state>
void pointLidarToWorld(PointType const &pi, PointType &po, const ikfom_state &state)
{
    V3D p_lidar(pi.x, pi.y, pi.z);
    V3D p_global(state.rot.normalized() * (state.offset_R_L_I.normalized() * p_lidar + state.offset_T_L_I) + state.pos);

    po.x = p_global(0);
    po.y = p_global(1);
    po.z = p_global(2);
    po.intensity = pi.intensity;
}

inline void pointLidarToWorld(PointType const &pi, PointType &po, const QD &lidar_rot, const V3D &lidar_pos)
{
    V3D p_lidar(pi.x, pi.y, pi.z);
    V3D p_global(lidar_rot.normalized() * p_lidar + lidar_pos);

    po.x = p_global(0);
    po.y = p_global(1);
    po.z = p_global(2);
    po.intensity = pi.intensity;
}

template <typename ikfom_state>
void pointcloudLidarToWorld(const PointCloudType::Ptr cloud_in, PointCloudType::Ptr cloud_out, const ikfom_state &state)
{
    auto cloud_num = cloud_in->points.size();
    cloud_out->resize(cloud_num);

    // imu pose -> lidar pose
    QD lidar_rot = state.rot * state.offset_R_L_I;
    V3D lidar_pos = state.rot * state.offset_T_L_I + state.pos;

#pragma omp parallel for num_threads(MP_PROC_NUM)
    for (int i = 0; i < cloud_num; i++)
    {
        pointLidarToWorld(cloud_in->points[i], cloud_out->points[i], lidar_rot, lidar_pos);
    }
}

inline PointCloudType::Ptr pointcloudKeyframeToWorld(const PointCloudType::Ptr &cloud_in, const PointXYZIRPYT &pose)
{
    int cloudSize = cloud_in->size();
    PointCloudType::Ptr cloud_out(new PointCloudType(cloudSize, 1));
    cloud_out->resize(cloudSize);

    const QD &state_rot = EigenMath::RPY2Quaternion(V3D(pose.roll, pose.pitch, pose.yaw));
    const V3D &state_pos = V3D(pose.x, pose.y, pose.z);

#pragma omp parallel for num_threads(MP_PROC_NUM)
    for (int i = 0; i < cloudSize; ++i)
    {
        V3D p_lidar(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
        V3D p_global(state_rot.normalized() * p_lidar + state_pos);
        cloud_out->points[i].x = p_global(0);
        cloud_out->points[i].y = p_global(1);
        cloud_out->points[i].z = p_global(2);
        cloud_out->points[i].intensity = cloud_in->points[i].intensity;
    }
    return cloud_out;
}

enum FrontendOdometryType
{
    Fastlio,
    Pointlio
};

#define NO_LOGER

class LogAnalysis
{
public:
    LogAnalysis()
    {
        frame_num = preprocess_time = preprocess_avetime = 0;
        imu_process_avetime = downsample_avetime = kdtree_search_avetime = match_avetime = cal_H_avetime = 0;
        meas_update_avetime = kdtree_incremental_avetime = kdtree_delete_avetime = map_incre_avetime = map_remove_avetime = total_avetime = 0;

#ifndef NO_LOGER
        fout_predict = fopen(DEBUG_FILE_DIR("state_predict.txt").c_str(), "w");
        fout_update = fopen(DEBUG_FILE_DIR("state_update.txt").c_str(), "w");
        fout_fastlio_log = fopen(DEBUG_FILE_DIR("fast_lio_log.csv").c_str(), "w");

        if (fout_predict && fout_update)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;
#endif
    }

    ~LogAnalysis()
    {
        fclose(fout_predict);
        fclose(fout_update);
        fclose(fout_fastlio_log);
    }

    template <typename ikfom_state>
    void print_pose(const ikfom_state &state, const std::string &print)
    {
        const auto &xyz = state.pos;
        const auto &rpy = EigenMath::Quaternion2RPY(state.rot);
        LOG_INFO("%s (xyz, rpy): (%.5f, %.5f, %.5f, %.5f, %.5f, %.5f)", print.c_str(), xyz(0), xyz(1), xyz(2), RAD2DEG(rpy(0)), RAD2DEG(rpy(1)), RAD2DEG(rpy(2)));
    }

    template <typename ikfom_state>
    void print_extrinsic(const ikfom_state &state, bool need_print)
    {
        const auto &offset_xyz = state.offset_T_L_I;
        const auto &offset_rpy = EigenMath::Quaternion2RPY(state.offset_R_L_I);
        LOG_INFO_COND(need_print, "extrinsic_est: (%.5f, %.5f, %.5f, %.5f, %.5f, %.5f)", offset_xyz(0), offset_xyz(1), offset_xyz(2), RAD2DEG(offset_rpy(0)), RAD2DEG(offset_rpy(1)), RAD2DEG(offset_rpy(2)));
    }

    void output_fastlio_log_to_csv(const double &lidar_beg_time)
    {
        if (!runtime_log)
            return;
        static bool first = true;
        if (first)
        {
            fprintf(fout_fastlio_log, "timestamp,total time,feats_undistort size,incremental time,search time,delete size,delete time,"
                                      "kdtree size,kdtree size end,add point size,preprocess time\n");
            first = false;
        }

        fprintf(fout_fastlio_log, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
                lidar_beg_time, total_time, feats_undistort_size, kdtree_incremental_time, kdtree_search_time,
                kdtree_delete_counter, kdtree_delete_time, kdtree_size, kdtree_size_end, add_point_size, preprocess_time);
    }

    void resetTimer()
    {
        timer.restart();

        imu_process_time = 0;
        downsample_time = 0;

        kdtree_search_time = 0;
        match_time = 0;
        cal_H_time = 0;
        meas_update_time = 0;

        kdtree_incremental_time = 0;
        kdtree_delete_time = 0;
        map_incre_time = 0;
        map_remove_time = 0;

        total_time = 0;

        feats_undistort_size = 0;
        feats_down_size = 0;
    }

    void print_fastlio_cost_time()
    {
        total_time = preprocess_time + imu_process_time + downsample_time + meas_update_time + map_incre_time + map_remove_time;

        if (!runtime_log)
            return;

        frame_num++;

        preprocess_avetime = (preprocess_avetime * (frame_num - 1) + preprocess_time) / frame_num;
        imu_process_avetime = (imu_process_avetime * (frame_num - 1) + imu_process_time) / frame_num;
        downsample_avetime = (downsample_avetime * (frame_num - 1) + downsample_time) / frame_num;

        kdtree_search_avetime = (kdtree_search_avetime * (frame_num - 1) + kdtree_search_time) / frame_num;
        match_avetime = (match_avetime * (frame_num - 1) + match_time) / frame_num;
        cal_H_avetime = (cal_H_avetime * (frame_num - 1) + cal_H_time) / frame_num;
        meas_update_avetime = (meas_update_avetime * (frame_num - 1) + meas_update_time) / frame_num;

        kdtree_incremental_avetime = (kdtree_incremental_avetime * (frame_num - 1) + kdtree_incremental_time) / frame_num;
        kdtree_delete_avetime = (kdtree_delete_avetime * (frame_num - 1) + kdtree_delete_time) / frame_num;
        map_incre_avetime = (map_incre_avetime * (frame_num - 1) + map_incre_time) / frame_num;
        map_remove_avetime = (map_remove_avetime * (frame_num - 1) + map_remove_time) / frame_num;

        total_avetime = (total_avetime * (frame_num - 1) + total_time) / frame_num;

#if 0
        printf("[ave_time]: feats_undistort: %d, feats_down: %d, preprocess: %0.3f, imu: %0.3f, downsample: %0.3f, search: %0.3f, match: %0.3f, "
               "meas update: %0.3f, map incre: %0.3f, map remove: %0.3f, ave total: %0.3f\n",
               feats_undistort_size, feats_down_size,
               preprocess_avetime, imu_process_avetime, downsample_avetime, kdtree_search_avetime, match_avetime,
               meas_update_avetime, map_incre_avetime, map_remove_avetime, total_avetime);
#else
        printf("[cur_time]: feats_undistort: %d, feats_down: %d, preprocess: %0.3f, imu: %0.3f, downsample: %0.3f, search: %0.3f, match: %0.3f, "
               "meas update: %0.3f, map incre: %0.3f, map remove: %0.3f, total: %0.3f\n",
               feats_undistort_size, feats_down_size,
               preprocess_time, imu_process_time, downsample_time, kdtree_search_time, match_time,
               meas_update_time, map_incre_time, map_remove_time, total_time);
#endif
    }

    // 0 : not, 1 : TUM
    static void save_trajectory(FILE *fp, const V3D &pos, const QD &quat, const double &time, int save_traj_fmt = 1)
    {
        if (save_traj_fmt == 1)
        {
            fprintf(fp, "%0.4lf %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", time,
                    pos.x(), pos.y(), pos.z(), quat.x(), quat.y(), quat.z(), quat.w());
        }

        fflush(fp);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int runtime_log = 0;
    bool inited_first_lidar_beg_time = false;
    double first_lidar_beg_time;

    FILE *fout_fastlio_log;
    FILE *fout_predict, *fout_update;
    Timer timer;

    long unsigned int frame_num;
    double preprocess_time, imu_process_time, downsample_time, kdtree_search_time, match_time, cal_H_time;
    double meas_update_time, kdtree_incremental_time, kdtree_delete_time, map_incre_time, map_remove_time, total_time;

    double preprocess_avetime, imu_process_avetime, downsample_avetime, kdtree_search_avetime, match_avetime, cal_H_avetime;
    double meas_update_avetime, kdtree_incremental_avetime, kdtree_delete_avetime, map_incre_avetime, map_remove_avetime, total_avetime;

    int feats_undistort_size = 0, feats_down_size = 0, kdtree_size = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
};


// #define DEDUB_MODE
