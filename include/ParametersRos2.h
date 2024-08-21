#pragma once
#include "frontend/FastlioOdometry.hpp"
#include "backend/backend/Backend.hpp"
#include <rclcpp/rclcpp.hpp>

inline void load_ros_parameters(rclcpp::Node::SharedPtr &node, bool &path_en, bool &scan_pub_en, bool &dense_pub_en,
                                std::string &lidar_topic, std::string &imu_topic, std::string &gnss_topic,
                                std::string &map_frame, std::string &body_frame, std::string &lidar_frame)
{
    node->declare_parameter("path_en", false);
    node->declare_parameter("scan_publish_en", false);
    node->declare_parameter("dense_publish_en", false);

    node->get_parameter("path_en", path_en);
    node->get_parameter("scan_publish_en", scan_pub_en);
    node->get_parameter("dense_publish_en", dense_pub_en);

    node->declare_parameter("lidar_topic", "/livox/lidar");
    node->declare_parameter("imu_topic", "/livox/imu");
    node->declare_parameter("gnss_topic", "/gps/fix");
    node->declare_parameter("map_frame", "camera_init");
    node->declare_parameter("body_frame", "body");
    node->declare_parameter("lidar_frame", "lidar");

    node->get_parameter("lidar_topic", lidar_topic);
    node->get_parameter("imu_topic", imu_topic);
    node->get_parameter("gnss_topic", gnss_topic);
    node->get_parameter("map_frame", map_frame);
    node->get_parameter("body_frame", body_frame);
    node->get_parameter("lidar_frame", lidar_frame);
}

inline void load_parameters(rclcpp::Node::SharedPtr &node, FastlioOdometry &frontend, Backend &backend, bool &save_globalmap_en, int &lidar_type)
{
    double blind, detect_range;
    int n_scans, scan_rate, time_unit;
    vector<double> extrinT;
    vector<double> extrinR;
    V3D extrinT_eigen;
    M3D extrinR_eigen;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;

    node->declare_parameter("keyframe_add_dist_threshold", 1.f);
    node->declare_parameter("keyframe_add_angle_threshold", 0.2f);
    node->declare_parameter("pose_cov_threshold", 25.f);
    node->declare_parameter("gnssValidInterval", 0.2f);
    node->declare_parameter("gpsCovThreshold", 2.f);
    node->declare_parameter("useGpsElevation", false);
    node->declare_parameter("extrinsic_gnss_T", vector<double>());
    node->declare_parameter("extrinsic_gnss_R", vector<double>());
    node->declare_parameter("recontruct_kdtree", true);
    node->declare_parameter("ikdtree_reconstruct_keyframe_num", 10);
    node->declare_parameter("ikdtree_reconstruct_downsamp_size", 0.1f);
    node->declare_parameter("loop_closure_enable_flag", false);
    node->declare_parameter("loop_closure_interval", 1000);
    node->declare_parameter("loop_keyframe_num_thld", 50);
    node->declare_parameter("loop_closure_search_radius", 10.f);
    node->declare_parameter("loop_closure_keyframe_interval", 30);
    node->declare_parameter("keyframe_search_num", 20);
    node->declare_parameter("loop_closure_fitness_score_thld", 0.05);
    node->declare_parameter("icp_downsamp_size", 0.1);
    node->declare_parameter("manually_loop_vaild_period", vector<double>());
    node->declare_parameter("odom_loop_vaild_period", vector<double>());
    node->declare_parameter("scancontext_loop_vaild_period", vector<double>());
    node->declare_parameter("gyr_cov", 0.1);
    node->declare_parameter("acc_cov", 0.1);
    node->declare_parameter("b_gyr_cov", 0.0001);
    node->declare_parameter("b_acc_cov", 0.0001);
    node->declare_parameter("blind", 0.01);
    node->declare_parameter("det_range", 300.);
    node->declare_parameter("lidar_type", 1);
    node->declare_parameter("scan_line", 16);
    node->declare_parameter("timestamp_unit", 2);
    node->declare_parameter("scan_rate", 10);
    node->declare_parameter("save_globalmap_en", true);
    node->declare_parameter("save_keyframe_en", true);
    node->declare_parameter("save_keyframe_descriptor_en", true);
    node->declare_parameter("save_resolution", 0.1f);
    node->declare_parameter("map_path", "");
    node->declare_parameter("lidar_height", 2.0);
    node->declare_parameter("sc_dist_thres", 0.5);

    node->get_parameter("keyframe_add_dist_threshold", backend.backend->keyframe_add_dist_threshold);
    node->get_parameter("keyframe_add_angle_threshold", backend.backend->keyframe_add_angle_threshold);
    node->get_parameter("pose_cov_threshold", backend.backend->pose_cov_threshold);
    node->get_parameter("gnssValidInterval", backend.gnss->gnssValidInterval);
    node->get_parameter("gpsCovThreshold", backend.gnss->gpsCovThreshold);
    node->get_parameter("useGpsElevation", backend.gnss->useGpsElevation);

    node->get_parameter("extrinsic_gnss_T", extrinT);
    node->get_parameter("extrinsic_gnss_R", extrinR);
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    backend.gnss->set_extrinsic(extrinT_eigen, extrinR_eigen);

    node->get_parameter("recontruct_kdtree", backend.backend->recontruct_kdtree);
    node->get_parameter("ikdtree_reconstruct_keyframe_num", backend.backend->ikdtree_reconstruct_keyframe_num);
    node->get_parameter("ikdtree_reconstruct_downsamp_size", backend.backend->ikdtree_reconstruct_downsamp_size);

    node->get_parameter("loop_closure_enable_flag", backend.loop_closure_enable_flag);
    node->get_parameter("loop_closure_interval", backend.loop_closure_interval);
    node->get_parameter("loop_keyframe_num_thld", backend.loopClosure->loop_keyframe_num_thld);
    node->get_parameter("loop_closure_search_radius", backend.loopClosure->loop_closure_search_radius);
    node->get_parameter("loop_closure_keyframe_interval", backend.loopClosure->loop_closure_keyframe_interval);
    node->get_parameter("keyframe_search_num", backend.loopClosure->keyframe_search_num);
    node->get_parameter("loop_closure_fitness_score_thld", backend.loopClosure->loop_closure_fitness_score_thld);
    node->get_parameter("icp_downsamp_size", backend.loopClosure->icp_downsamp_size);
    node->get_parameter("manually_loop_vaild_period", backend.loopClosure->loop_vaild_period["manually"]);
    node->get_parameter("odom_loop_vaild_period", backend.loopClosure->loop_vaild_period["odom"]);
    node->get_parameter("scancontext_loop_vaild_period", backend.loopClosure->loop_vaild_period["scancontext"]);

    node->get_parameter("gyr_cov", gyr_cov);
    node->get_parameter("acc_cov", acc_cov);
    node->get_parameter("b_gyr_cov", b_gyr_cov);
    node->get_parameter("b_acc_cov", b_acc_cov);
    node->get_parameter("blind", blind);
    node->get_parameter("det_range", detect_range);
    node->get_parameter("lidar_type", lidar_type);
    node->get_parameter("scan_line", n_scans);
    node->get_parameter("timestamp_unit", time_unit);
    node->get_parameter("scan_rate", scan_rate);
    node->get_parameter("save_globalmap_en", save_globalmap_en);
    node->get_parameter("save_keyframe_en", backend.save_keyframe_en);
    node->get_parameter("save_keyframe_descriptor_en", backend.save_keyframe_descriptor_en);
    node->get_parameter("save_resolution", backend.save_resolution);
    node->get_parameter("map_path", backend.map_path);
    if (backend.map_path.compare("") != 0)
    {
        backend.globalmap_path = backend.map_path + "/globalmap.pcd";
        backend.trajectory_path = backend.map_path + "/trajectory.pcd";
        backend.keyframe_path = backend.map_path + "/keyframe/";
        backend.scd_path = backend.map_path + "/scancontext/";
    }
    else
        backend.map_path = PCD_FILE_DIR("");

    node->get_parameter("lidar_height", backend.relocalization->sc_manager->LIDAR_HEIGHT);
    node->get_parameter("sc_dist_thres", backend.relocalization->sc_manager->SC_DIST_THRES);

    if (false)
    {
        node->declare_parameter("utm_origin_zone", "51N");
        node->declare_parameter("utm_origin_east", 0.);
        node->declare_parameter("utm_origin_north", 0.);
        node->declare_parameter("utm_origin_up", 0.);
        node->declare_parameter("extrinsicT_imu2gnss", vector<double>());
        node->declare_parameter("extrinsicR_imu2gnss", vector<double>());
        node->declare_parameter("relocal_cfg_algorithm_type", "UNKONW");

        node->get_parameter("utm_origin_zone", backend.relocalization->utm_origin.zone);
        node->get_parameter("utm_origin_east", backend.relocalization->utm_origin.east);
        node->get_parameter("utm_origin_north", backend.relocalization->utm_origin.north);
        node->get_parameter("utm_origin_up", backend.relocalization->utm_origin.up);

        node->get_parameter("extrinsicT_imu2gnss", extrinT);
        node->get_parameter("extrinsicR_imu2gnss", extrinR);
        extrinT_eigen << VEC_FROM_ARRAY(extrinT);
        extrinR_eigen << MAT_FROM_ARRAY(extrinR);
        backend.relocalization->set_extrinsic(extrinT_eigen, extrinR_eigen);

        node->get_parameter("relocal_cfg_algorithm_type", backend.relocalization->algorithm_type);

        BnbOptions match_option;
        node->declare_parameter("bnb3d_linear_xy_window_size", 10.);
        node->declare_parameter("bnb3d_linear_z_window_size", 1.);
        node->declare_parameter("bnb3d_angular_search_window", 30.);
        node->declare_parameter("bnb3d_pc_resolutions", vector<double>());
        node->declare_parameter("bnb3d_bnb_depth", 5);
        node->declare_parameter("bnb3d_min_score", 0.1);
        node->declare_parameter("bnb3d_enough_score", 0.8);
        node->declare_parameter("bnb3d_min_xy_resolution", 0.2);
        node->declare_parameter("bnb3d_min_z_resolution", 0.1);
        node->declare_parameter("bnb3d_min_angular_resolution", 0.1);
        node->declare_parameter("bnb3d_filter_size_scan", 0.1);
        node->declare_parameter("bnb3d_debug_mode", false);

        node->get_parameter("bnb3d_linear_xy_window_size", match_option.linear_xy_window_size);
        node->get_parameter("bnb3d_linear_z_window_size", match_option.linear_z_window_size);
        node->get_parameter("bnb3d_angular_search_window", match_option.angular_search_window);
        node->get_parameter("bnb3d_pc_resolutions", match_option.pc_resolutions);
        node->get_parameter("bnb3d_depth", match_option.bnb_depth);
        node->get_parameter("bnb3d_min_score", match_option.min_score);
        node->get_parameter("bnb3d_enough_score", match_option.enough_score);
        node->get_parameter("bnb3d_min_xy_resolution", match_option.min_xy_resolution);
        node->get_parameter("bnb3d_min_z_resolution", match_option.min_z_resolution);
        node->get_parameter("bnb3d_min_angular_resolution", match_option.min_angular_resolution);
        node->get_parameter("bnb3d_filter_size_scan", match_option.filter_size_scan);
        node->get_parameter("bnb3d_debug_mode", match_option.debug_mode);

        node->declare_parameter("extrinsic_T", vector<double>());
        node->declare_parameter("extrinsic_R", vector<double>());
        node->get_parameter("extrinsic_T", extrinT);
        node->get_parameter("extrinsic_R", extrinR);
        extrinT_eigen << VEC_FROM_ARRAY(extrinT);
        extrinR_eigen << MAT_FROM_ARRAY(extrinR);
        V3D ext_rpy = EigenMath::RotationMatrix2RPY(extrinR_eigen);
        Pose lidar_extrinsic;
        lidar_extrinsic.x = extrinT_eigen.x();
        lidar_extrinsic.y = extrinT_eigen.y();
        lidar_extrinsic.z = extrinT_eigen.z();
        lidar_extrinsic.roll = ext_rpy.x();
        lidar_extrinsic.pitch = ext_rpy.y();
        lidar_extrinsic.yaw = ext_rpy.z();
        backend.relocalization->set_bnb3d_param(match_option, lidar_extrinsic);

        double step_size, resolution;
        node->declare_parameter("ndt_step_size", 0.1);
        node->declare_parameter("ndt_resolution", 1.);
        node->get_parameter("ndt_step_size", step_size);
        node->get_parameter("ndt_resolution", resolution);
        backend.relocalization->set_ndt_param(step_size, resolution);

        bool use_gicp;
        double gicp_downsample, filter_range, search_radius, teps, feps, fitness_score;
        node->declare_parameter("gicp_use_gicp", true);
        node->declare_parameter("gicp_filter_range", 80.);
        node->declare_parameter("gicp_downsample", 0.2);
        node->declare_parameter("gicp_search_radius", 0.5);
        node->declare_parameter("gicp_teps", 1e-3);
        node->declare_parameter("gicp_feps", 1e-3);
        node->declare_parameter("gicp_fitness_score", 0.3);

        node->get_parameter("gicp_use_gicp", use_gicp);
        node->get_parameter("gicp_filter_range", filter_range);
        node->get_parameter("gicp_gicp_downsample", gicp_downsample);
        node->get_parameter("gicp_search_radius", search_radius);
        node->get_parameter("gicp_teps", teps);
        node->get_parameter("gicp_feps", feps);
        node->get_parameter("gicp_fitness_score", fitness_score);
        backend.relocalization->set_gicp_param(use_gicp, filter_range, gicp_downsample, search_radius, teps, feps, fitness_score);
    }

    frontend.lidar->init(n_scans, scan_rate, time_unit, blind);
    frontend.imu->set_imu_cov(process_noise_cov(gyr_cov, acc_cov, b_gyr_cov, b_acc_cov));

    vector<double> gravity_init, eular_init;
    node->declare_parameter("timedelay_lidar2imu", 0.f);
    node->declare_parameter("gravity_align", true);
    node->declare_parameter("gravity_init", vector<double>());
    node->declare_parameter("eular_init", vector<double>());
    node->get_parameter("timedelay_lidar2imu", frontend.timedelay_lidar2imu);
    node->get_parameter("gravity_align", frontend.gravity_align);
    node->get_parameter("gravity_init", gravity_init);
    node->get_parameter("eular_init", eular_init);
    frontend.preset_gravity << VEC_FROM_ARRAY(gravity_init);
    V3D rpy_init;
    rpy_init << VEC_FROM_ARRAY(eular_init);
    rpy_init *= M_PI / 180;
    frontend.imu_init_rot = EigenMath::RPY2Quaternion(rpy_init);

    node->declare_parameter("max_iteration", 4);
    node->declare_parameter("surf_frame_ds_res", 0.5);
    node->declare_parameter("point_skip_num", 2);
    node->declare_parameter("space_down_sample", true);
    node->declare_parameter("ikdtree_resolution", 0.5);
    node->declare_parameter("lidar_model_search_range", 5.);
    node->declare_parameter("lidar_meas_cov", 0.001);
    node->declare_parameter("cube_side_length", 200.);
    node->declare_parameter("extrinsic_est_en", true);
    node->declare_parameter("runtime_log_enable", 0);
    node->get_parameter("max_iteration", frontend.num_max_iterations);
    node->get_parameter("surf_frame_ds_res", frontend.surf_frame_ds_res);
    node->get_parameter("point_skip_num", frontend.point_skip_num);
    node->get_parameter("space_down_sample", frontend.space_down_sample);
    node->get_parameter("ikdtree_resolution", frontend.ikdtree_resolution);
    node->get_parameter("lidar_model_search_range", frontend.lidar_model_search_range);
    node->get_parameter("lidar_meas_cov", frontend.lidar_meas_cov);
    node->get_parameter("cube_side_length", frontend.cube_len);
    node->get_parameter("extrinsic_est_en", frontend.extrinsic_est_en);
    node->get_parameter("runtime_log_enable", frontend.loger.runtime_log);

    node->declare_parameter("extrinsic_T", vector<double>());
    node->declare_parameter("extrinsic_R", vector<double>());
    node->get_parameter("extrinsic_T", extrinT);
    node->get_parameter("extrinsic_R", extrinR);
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    frontend.set_extrinsic(extrinT_eigen, extrinR_eigen);
    frontend.init_estimator();

    node->declare_parameter("ground_constraint_enable", false);
    node->declare_parameter("ground_constraint_angle", 5.f);
    node->get_parameter("ground_constraint_enable", frontend.ground_constraint_enable);
    node->get_parameter("ground_constraint_angle", frontend.ground_constraint_angle);
    backend.init_system_mode();
}

inline void load_pgm_parameters(rclcpp::Node::SharedPtr &node, bool &save_pgm, double &pgm_resolution, float &min_z, float &max_z)
{
    node->declare_parameter("save_pgm", false);
    node->declare_parameter("pgm_resolution", 0.05);
    node->declare_parameter("min_z", -1.5f);
    node->declare_parameter("max_z", 0.1f);

    node->get_parameter("save_pgm", save_pgm);
    node->get_parameter("pgm_resolution", pgm_resolution);
    node->get_parameter("min_z", min_z);
    node->get_parameter("max_z", max_z);
}
