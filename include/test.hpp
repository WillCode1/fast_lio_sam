#pragma once
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "ImuProcessor.h"
#include "LidarProcessor.hpp"
#include "utility/Header.h"


class FastlioOdometry
{
public:
    FastlioOdometry() : frontend_type(Fastlio)
    {
        surf_frame_ds_res = 0.5;
        point_skip_num = 10;
        ikdtree_resolution = 0.5;
        cube_len = 200;
        detect_range = 300;

        feats_down_lidar.reset(new PointCloudType());
        feats_down_world.reset(new PointCloudType());
        normvec.reset(new PointCloudType());

        lidar = make_shared<LidarProcessor>();
        imu = make_shared<ImuProcessor>();
        measures = make_shared<MeasureCollection>();
    }

    // https://blog.csdn.net/u010949023/article/details/121248834
    // 更多的约束条件，可能会导致高斯牛顿求解失败
    void lidar_ground_constraint(state_ikfom &s2, esekfom::fastlio_ground_constraint_datastruct<double> &ekfom_data)
    {
        esekfom::fastlio_datastruct<double> tmp;
        tmp.valid = ekfom_data.valid;
        tmp.converge = ekfom_data.converge;
        lidar_meas_model(s2, tmp);
        ekfom_data.valid = tmp.valid;
        ekfom_data.converge = tmp.converge;
        ekfom_data.H = tmp.H;
        ekfom_data.z = tmp.z;
        ekfom_data.R = tmp.R;
        /************************/

        ekfom_data.r = MatrixXd::Zero(1, 3);
        ekfom_data.G = MatrixXd::Zero(3, 12);

        Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 3);
        const state_ikfom &s1 = state_pre;

        Eigen::Quaterniond R1T = s1.rot.conjugate() * s1.offset_R_L_I;
        Eigen::Quaterniond R2 = s2.offset_R_L_I.conjugate() * s2.rot;
        Eigen::Vector3d p1 = s1.offset_R_L_I.conjugate() * (s1.pos - s1.offset_T_L_I);
        Eigen::Vector3d p2 = s2.offset_R_L_I.conjugate() * (s2.pos - s2.offset_T_L_I);

        /*** calculate residual ***/
        ekfom_data.r.block<1, 1>(0, 0) = -e3.transpose() * (R1T * (p2 - p1));     // z
        ekfom_data.r.block<1, 2>(0, 1) = -A * (R1T * R2).toRotationMatrix() * e3; // roll, pitch

        /*** calculate Jacobian matrix H ***/
        if (extrinsic_est_en)
        {
            const auto &R1T_tmp = R1T * s2.offset_R_L_I.conjugate();
            ekfom_data.G.block<1, 3>(0, 0) = -e3.transpose() * R1T_tmp.toRotationMatrix();                                                                    // z/pos
            ekfom_data.G.block<1, 3>(0, 3) = Eigen::MatrixXd::Zero(1, 3);                                                                                     // z/rot
            ekfom_data.G.block<1, 3>(0, 6) = -e3.transpose() * (R1T * SO3Math::get_skew_symmetric(s2.offset_R_L_I.conjugate() * (s2.pos - s2.offset_T_L_I))); // z/ext_R
            ekfom_data.G.block<1, 3>(0, 9) = e3.transpose() * R1T_tmp.toRotationMatrix();                                                                     // z/ext_t

            ekfom_data.G.block<2, 3>(1, 0) = Eigen::MatrixXd::Zero(2, 3);                                                                          // roll, pitch/pos
            ekfom_data.G.block<2, 3>(1, 3) = A * (R1T * R2).toRotationMatrix() * SO3Math::get_skew_symmetric(e3);                                  // roll, pitch/rot
            ekfom_data.G.block<2, 3>(1, 6) = -A * R1T.toRotationMatrix() * SO3Math::get_skew_symmetric(s2.offset_R_L_I.conjugate() * s2.rot * e3); // roll, pitch/ext_R
            ekfom_data.G.block<2, 3>(1, 9) = Eigen::MatrixXd::Zero(2, 3);                                                                          // roll, pitch/ext_t
        }
        else
        {
            ekfom_data.G.block<1, 3>(0, 0) = -e3.transpose() * s1.rot.conjugate().toRotationMatrix();                                // z/pos
            ekfom_data.G.block<1, 3>(0, 3) = Eigen::MatrixXd::Zero(1, 3);                                                            // z/rot
            ekfom_data.G.block<2, 3>(1, 0) = Eigen::MatrixXd::Zero(2, 3);                                                            // roll, pitch/pos
            ekfom_data.G.block<2, 3>(1, 3) = A * (s1.rot.conjugate() * s2.rot).toRotationMatrix() * SO3Math::get_skew_symmetric(e3); // roll, pitch/rot
        }
        ekfom_data.N = 0.0001;
    }

protected:
    struct EffectFeature
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        V3D point_lidar;
        V3D norm_vec;
        double residual;
    };

    vector<bool> point_matched_surface;
    vector<PointVector> nearest_points;
    PointCloudType::Ptr normvec;
    std::vector<EffectFeature> effect_features;

public:
    bool extrinsic_est_en = false;
    /*** for gravity align ***/
    bool map_rotate = true;
    std::vector<double> eular_init;

    /*** backup for relocalization reset ***/
    V3D offset_Tli;
    M3D offset_Rli;
    V3D gravity_init;
    LogAnalysis loger;

    /*** sensor data processor ***/
    shared_ptr<LidarProcessor> lidar;
    shared_ptr<ImuProcessor> imu;
    shared_ptr<MeasureCollection> measures;
    deque<ImuData::Ptr> imu_buffer;
    deque<double> time_buffer;
    deque<PointCloudType::Ptr> lidar_buffer;

    double latest_lidar_beg_time = 0;
    double latest_timestamp_imu = -1.0;
    double timedelay_lidar2imu = 0.0;
    double lidar_end_time = 0;
    mutex mtx_buffer;

    int frontend_type;
    /*** frontend odometry ***/
    int point_skip_num;
    bool space_down_sample = true;
    double surf_frame_ds_res;
    pcl::VoxelGrid<PointType> surf_frame_ds_filter;
    int feats_down_size = 0;
    PointCloudType::Ptr feats_down_lidar;
    PointCloudType::Ptr feats_down_world;

    /*** ESKF inputs and output ***/
    int num_max_iterations = 4;
    const int NUM_MATCH_POINTS = 5;
    double lidar_model_search_range = 5;
    double lidar_meas_cov = 0.001;

    /*** local map maintain ***/
    bool localmap_initialized = false;
    const float move_threshold = 1.5f;
    double cube_len;
    double detect_range;
    BoxPointType local_map_bbox;
    double ikdtree_resolution;
    KD_TREE<PointType> ikdtree;

private:
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state;
};
