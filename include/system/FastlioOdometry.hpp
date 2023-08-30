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
#include "utility/Header.h"


class FastlioOdometry
{
public:
    FastlioOdometry()
    {
        surf_frame_ds_res = 0.5;
        point_skip_num = 10;
        ikdtree_resolution = 0.5;
        cube_len = 200;
        detect_range = 300;

        feats_down_lidar.reset(new PointCloudType());
        normvec.reset(new PointCloudType());
    }

    void init_global_map(PointCloudType::Ptr& global_map)
    {
        if (ikdtree.Root_Node != nullptr)
        {
            LOG_ERROR("Error, ikdtree not null when initializing the map!");
            std::exit(100);
        }
        ikdtree.set_downsample_param(ikdtree_resolution);
        ikdtree.Build(global_map->points);
    }

    void set_extrinsic(const V3D &transl, const M3D &rot, const V3D &gravity = V3D(0, 0, -G_m_s2))
    {
        state = kf.get_x();
        state.offset_T_L_I = transl;
        state.offset_R_L_I = rot;
        state.grav.vec = gravity;
        kf.change_x(state);

        offset_Tli = transl;
        offset_Rli = rot;
        gravity_vec = gravity;
    }

    bool run(bool localization_mode, shared_ptr<ImuProcessor> &imu, MeasureCollection &measures,
             PointCloudType::Ptr &feats_undistort, LogAnalysis &loger)
    {
        imu->Process(measures, kf, feats_undistort, localization_mode);
        state = kf.get_x();
        loger.imu_process_time = loger.timer.elapsedLast();
        loger.feats_undistort_size = feats_undistort->points.size();

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            LOG_WARN("No point, skip this scan!");
            return true;
        }

        // TODO: compare strategy
        // lasermap_fov_segment(loger);

        /*** interval sample and downsample the feature points in a scan ***/
        feats_down_lidar->clear();
        for (int i = 0; i < feats_undistort->size(); i++)
            if (i % point_skip_num == 0)
            {
                feats_down_lidar->points.push_back(feats_undistort->points[i]);
            }
        surf_frame_ds_filter.setLeafSize(surf_frame_ds_res, surf_frame_ds_res, surf_frame_ds_res);
        surf_frame_ds_filter.setInputCloud(feats_down_lidar);
        surf_frame_ds_filter.filter(*feats_down_lidar);
        feats_down_size = feats_down_lidar->points.size();
        loger.feats_down_size = feats_down_size;
        loger.downsample_time = loger.timer.elapsedLast();

        /*** initialize the map kdtree ***/
        if (ikdtree.Root_Node == nullptr)
        {
            if (localization_mode)
            {
                LOG_ERROR("localization mode, please load map before!");
                std::exit(100);
            }
            else if (feats_down_size > 5)
            {
                ikdtree.set_downsample_param(ikdtree_resolution);
                PointCloudType::Ptr feats_down_world(new PointCloudType);
                pointcloudLidarToWorld(feats_down_lidar, feats_down_world, state);
                ikdtree.Build(feats_down_world->points);
            }
            return false;
        }
        loger.kdtree_size = ikdtree.size();
        loger.output2file(state, loger.fout_predict, measures.lidar_beg_time - loger.first_lidar_beg_time);

        /*** iterated state estimation ***/
        point_matched_surface.resize(feats_down_size);
        nearest_points.resize(feats_down_size);
        normvec->resize(feats_down_size);
        if (!kf.update_iterated_dyn_share_modified(LASER_POINT_COV, loger.iterate_ekf_time))
        {
            LOG_WARN("Lidar degradation!");
            return false;
        }
        state = kf.get_x();
        loger.meas_update_time = loger.timer.elapsedLast();

        loger.output2file(state, loger.fout_update, measures.lidar_beg_time - loger.first_lidar_beg_time);
        loger.dump_state_to_log(state, measures.lidar_beg_time - loger.first_lidar_beg_time);

        if (!localization_mode)
        {
            if (extrinsic_est_en)
            {
                const auto& offset_xyz = state.offset_T_L_I;
                const auto& offset_rpy = EigenMath::Quaternion2RPY(state.offset_R_L_I);
                LOG_INFO_COND(false, "extrinsic_est: (%.5f, %.5f, %.5f, %.5f, %.5f, %.5f)", offset_xyz(0), offset_xyz(1), offset_xyz(2), RAD2DEG(offset_rpy(0)), RAD2DEG(offset_rpy(1)), RAD2DEG(offset_rpy(2)));
            }
            /*** map update ***/
            lasermap_fov_segment(loger);
            loger.map_remove_time = loger.timer.elapsedLast();
            map_incremental(loger);
            loger.map_incre_time = loger.timer.elapsedLast();
            loger.kdtree_size_end = ikdtree.size();
        }
        else
        {
            // TODO: location mode dongtai load submap, and when not in raw map, could add new scan into ikdtree.
        }
        return true;
    }

    // 计算lidar point-to-plane Jacobi和残差
    void lidar_meas_model(state_ikfom &state, esekfom::dyn_share_datastruct<double> &ekfom_data, LogAnalysis& loger)
    {
        double match_start = omp_get_wtime();
        normvec->clear();
        effect_features.clear();

        double search_start = omp_get_wtime();
        /** closest surface search and residual computation **/
#ifdef MP_EN
#pragma omp parallel for num_threads(MP_PROC_NUM)
#endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType point = feats_down_lidar->points[i];

            /* transform to world frame */
            V3D p_lidar(point.x, point.y, point.z);
            V3D p_world = state.rot * (state.offset_R_L_I * p_lidar + state.offset_T_L_I) + state.pos;
            point.x = p_world.x();
            point.y = p_world.y();
            point.z = p_world.z(); 

            auto &points_near = nearest_points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                ikdtree.Nearest_Search(point, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_matched_surface[i] =
                    points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > lidar_model_search_range ? false
                                                                                                                                      : true;
            }

            if (!point_matched_surface[i])
                continue;

            Eigen::Vector4d abcd;
            point_matched_surface[i] = false;
            if (esti_plane(abcd, points_near, 0.1f))
            {
                float dis = abcd(0) * point.x + abcd(1) * point.y + abcd(2) * point.z + abcd(3);
                float s = 1 - 0.9 * fabs(dis) / sqrt(p_lidar.norm());

                if (s > 0.9)
                {
                    point_matched_surface[i] = true;
                    normvec->points[i].x = abcd(0);
                    normvec->points[i].y = abcd(1);
                    normvec->points[i].z = abcd(2);
                    normvec->points[i].intensity = dis;
                }
            }
        }
        loger.kdtree_search_time += (omp_get_wtime() - search_start) * 1000;

        // omp中无法push_back
        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_matched_surface[i])
            {
                EffectFeature effect_feat;
                effect_feat.point_lidar = V3D(feats_down_lidar->points[i].x, feats_down_lidar->points[i].y, feats_down_lidar->points[i].z);
                effect_feat.norm_vec = V3D(normvec->points[i].x, normvec->points[i].y, normvec->points[i].z);
                effect_feat.residual = normvec->points[i].intensity;
                effect_features.emplace_back(effect_feat);
            }
        }

        int effct_feat_num = effect_features.size();
        if (effct_feat_num < 1)
        {
            ekfom_data.valid = false;
            LOG_WARN("No Effective Points!");
            return;
        }

        loger.match_time += (omp_get_wtime() - match_start) * 1000;
        double solve_start = omp_get_wtime();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
        ekfom_data.h.resize(effct_feat_num);

        // 求观测值与误差的雅克比矩阵，如论文式14以及式12、13
        for (int i = 0; i < effct_feat_num; i++)
        {
            const V3D& point_lidar = effect_features[i].point_lidar;
            const M3D& point_be_crossmat = SO3Math::get_skew_symmetric(point_lidar);
            const V3D& point_imu = state.offset_R_L_I * point_lidar + state.offset_T_L_I;
            const M3D& point_crossmat = SO3Math::get_skew_symmetric(point_imu);

            /*** get the normal vector of closest surface ***/
            const V3D &norm_vec = effect_features[i].norm_vec;

            /*** calculate the Measuremnt Jacobian matrix H ***/
            // 雅各比矩阵分子布局和分母布局的区别：(AR^Tu)^T = u^TR(A^T) = u^TR(-A)
            V3D C(state.rot.conjugate() * norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * state.offset_R_L_I.conjugate() * C);
                ekfom_data.h_x.block<1, 12>(i, 0) << VEC_FROM_ARRAY(norm_vec), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i, 0) << VEC_FROM_ARRAY(norm_vec), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest plane ***/
            ekfom_data.h(i) = -effect_features[i].residual;
        }
        loger.cal_H_time += (omp_get_wtime() - solve_start) * 1000;
    }

    void get_ikdtree_point(PointCloudType::Ptr& res)
    {
        PointVector().swap(ikdtree.PCL_Storage);
        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
        res->points = ikdtree.PCL_Storage;
    }

private:
    /**
     * 动态调整地图:
     * 1.初始化局部地图包围盒角点(首次)
     * 2.判断是否需要move
     * 3.需要就更新包围盒，并用ikdtree删除不需要的包围盒
     */
    void lasermap_fov_segment(LogAnalysis& loger)
    {
        loger.kdtree_delete_counter = 0; // for debug
        vector<BoxPointType> cub_needrm;

        V3D pos_Lidar_world = state.pos + state.rot * state.offset_T_L_I;
        // 初始化局部地图包围盒角点，以为w系下lidar位置为中心,得到长宽高200*200*200的局部地图
        if (!localmap_initialized)
        { 
            for (int i = 0; i < 3; i++)
            {
                local_map_bbox.vertex_min[i] = pos_Lidar_world(i) - cube_len / 2.0;
                local_map_bbox.vertex_max[i] = pos_Lidar_world(i) + cube_len / 2.0;
            }
            localmap_initialized = true;
            return;
        }
        // 各个方向上Lidar与局部地图边界的距离，或者说是lidar与立方体盒子六个面的距离
        float dist_to_map_edge[3][2];
        bool need_move = false;
        // 当前雷达系中心到各个地图边缘的距离
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_Lidar_world(i) - local_map_bbox.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_Lidar_world(i) - local_map_bbox.vertex_max[i]);
            // 与某个方向上的边界距离太小，标记需要移除need_move，参考论文Fig3
            if (dist_to_map_edge[i][0] <= move_threshold * detect_range || dist_to_map_edge[i][1] <= move_threshold * detect_range)
                need_move = true;
        }
        if (!need_move)
            return;
        // 否则需要计算移动的距离
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        // 新的局部地图盒子边界点
        New_LocalMap_Points = local_map_bbox;
        float mov_dist = max((cube_len - 2.0 * move_threshold * detect_range) * 0.5 * 0.9, double(detect_range * (move_threshold - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = local_map_bbox;
            if (dist_to_map_edge[i][0] <= move_threshold * detect_range)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = local_map_bbox.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints); // 移除较远包围盒
            }
            else if (dist_to_map_edge[i][1] <= move_threshold * detect_range)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = local_map_bbox.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        local_map_bbox = New_LocalMap_Points;

        double delete_begin = omp_get_wtime();
        if (cub_needrm.size() > 0)
            loger.kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
        loger.kdtree_delete_time = (omp_get_wtime() - delete_begin) * 1000;
    }

    void map_incremental(LogAnalysis& loger)
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        PointType feature_world;
        for (int i = 0; i < feats_down_size; i++)
        {
            /* transform to world frame */
            pointLidarToWorld(&(feats_down_lidar->points[i]), &feature_world, state);
            /* decide if need add to map */
            if (!nearest_points[i].empty())
            {
                const PointVector &points_near = nearest_points[i];
                bool need_add = true;
                BoxPointType Box_of_Point;
                PointType downsample_result, mid_point;
                mid_point.x = floor(feature_world.x / ikdtree_resolution) * ikdtree_resolution + 0.5 * ikdtree_resolution;
                mid_point.y = floor(feature_world.y / ikdtree_resolution) * ikdtree_resolution + 0.5 * ikdtree_resolution;
                mid_point.z = floor(feature_world.z / ikdtree_resolution) * ikdtree_resolution + 0.5 * ikdtree_resolution;
                float dist = pointDistanceSquare(feature_world, mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * ikdtree_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * ikdtree_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * ikdtree_resolution)
                {
                    PointNoNeedDownsample.push_back(feature_world);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
                {
                    if (points_near.size() < NUM_MATCH_POINTS)
                        break;
                    if (pointDistanceSquare(points_near[readd_i], mid_point) < dist)
                    {
                        need_add = false;
                        break;
                    }
                }
                if (need_add)
                    PointToAdd.push_back(feature_world);
            }
            else
            {
                PointToAdd.push_back(feature_world);
            }
        }

        double st_time = omp_get_wtime();
        loger.add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        loger.add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
        loger.kdtree_incremental_time = (omp_get_wtime() - st_time) * 1000;
    }

    bool esti_plane(Vector4d &pca_result, const PointVector &point, const double &threshold)
    {
        int num_match_point = point.size();
        MatrixXd A(num_match_point, 3);
        MatrixXd b(num_match_point, 1);
        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < num_match_point; j++)
        {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        V3D normvec = A.colPivHouseholderQr().solve(b);

        auto n = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;

        for (int j = 0; j < num_match_point; j++)
        {
            if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
            {
                return false;
            }
        }
        return true;
    }

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

    /*** backup for relocalization ***/
    V3D offset_Tli;
    M3D offset_Rli;
    V3D gravity_vec;

    /*** frontend odometry ***/
    int point_skip_num;
    double surf_frame_ds_res;
    pcl::VoxelGrid<PointType> surf_frame_ds_filter;
    int feats_down_size = 0;
    PointCloudType::Ptr feats_down_lidar;

    /*** ESKF inputs and output ***/
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state;
    int num_max_iterations = 4;
    const int NUM_MATCH_POINTS = 5;
    double lidar_model_search_range = 5;
    const double LASER_POINT_COV = 0.001;

    /*** local map maintain ***/
    bool localmap_initialized = false;
    const float move_threshold = 1.5f;
    double cube_len;
    double detect_range;
    BoxPointType local_map_bbox;
    double ikdtree_resolution;
    KD_TREE<PointType> ikdtree;
};
