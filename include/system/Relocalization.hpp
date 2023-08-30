#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include "utility/Header.h"
#include "global_localization/bnb3d.h"
#include "global_localization/scancontext/Scancontext.h"


class Relocalization
{
public:
    Relocalization();
    ~Relocalization();
    bool load_prior_map(const PointCloudType::Ptr &global_map);
    bool load_keyframe_descriptor(const std::string &path);
    bool run(const PointCloudType::Ptr &scan, Eigen::Matrix4d &result);

    void set_init_pose(const Pose &_manual_pose);
    void set_bnb3d_param(const BnbOptions &match_option, const Pose &lidar_pose);
    void set_plane_extract_param(const double &fr, const int &min_point, const double &clust_dis, const double &pl_dis, const double &point_percent);
    void set_gicp_param(const double &gicp_ds, const double &search_radi, const double &tep, const double &fep, const double &fit_score);
    void add_scancontext_descriptor(const PointCloudType::Ptr thiskeyframe, const std::string &path);

    std::string algorithm_type = "UNKNOW";
    BnbOptions bnb_option;
    Pose manual_pose, lidar_extrinsic, rough_pose;
    std::shared_ptr<BranchAndBoundMatcher3D> bnb3d;

    pcl::PointCloud<PointXYZIRPYT>::Ptr trajectory_poses;
    std::shared_ptr<ScanContext::SCManager> sc_manager; // scan context

private:
    PointCloudType::Ptr plane_seg(PointCloudType::Ptr src);
    bool plane_estimate(const pcl::PointCloud<PointType>::Ptr cluster, const float &threshold);
    bool fine_tune_pose(PointCloudType::Ptr scan, Eigen::Matrix4d &result, const Eigen::Matrix4d &lidar_ext);
    bool run_scan_context(PointCloudType::Ptr scan, Eigen::Matrix4d &rough_mat, const Eigen::Matrix4d &lidar_ext);
    bool run_manually_set(PointCloudType::Ptr scan, Eigen::Matrix4d &rough_mat, const Eigen::Matrix4d &lidar_ext);

    bool prior_pose_inited = false;

	// plane_seg
    double filter_radius = 1;
    int min_plane_point = 10;
    double cluster_dis = 0.1;
    double plane_dis = 0.2;
    double plane_point_percent = 0.1;

	// gicp
    double gicp_downsample = 0.2;
    double search_radius = 0.2;
    double teps = 0.001;
    double feps = 0.001;
    double fitness_score = 0.3;

    pcl::VoxelGrid<PointType> voxel_filter;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
};

Relocalization::Relocalization()
{
    sc_manager = std::make_shared<ScanContext::SCManager>();
    trajectory_poses.reset(new pcl::PointCloud<PointXYZIRPYT>());
}

Relocalization::~Relocalization()
{
}

bool Relocalization::run_scan_context(PointCloudType::Ptr scan, Eigen::Matrix4d &rough_mat, const Eigen::Matrix4d &lidar_ext)
{
    Timer timer;
    PointCloudType::Ptr scanDS(new PointCloudType());
    pcl::PointCloud<SCPointType>::Ptr sc_input(new pcl::PointCloud<SCPointType>());
    voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    voxel_filter.setInputCloud(scan);
    voxel_filter.filter(*scanDS);

    pcl::PointXYZI tmp;
    for (auto &point : scanDS->points)
    {
        tmp.x = point.x;
        tmp.y = point.y;
        tmp.z = point.z;
        sc_input->push_back(tmp);
    }
    auto sc_res = sc_manager->relocalize(*sc_input);
    if (sc_res.first != -1 && sc_res.first < trajectory_poses->size())
    {
        const auto &pose_ref = trajectory_poses->points[sc_res.first];
        // lidar pose -> imu pose
        rough_mat = EigenMath::CreateAffineMatrix(V3D(pose_ref.x, pose_ref.y, pose_ref.z), V3D(pose_ref.roll, pose_ref.pitch, pose_ref.yaw + sc_res.second));
        rough_mat *= lidar_ext.inverse();
        EigenMath::DecomposeAffineMatrix(rough_mat, rough_pose.x, rough_pose.y, rough_pose.z, rough_pose.roll, rough_pose.pitch, rough_pose.yaw);
        LOG_WARN("scan context success! res index = %d, pose = (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf)!", sc_res.first,
                 rough_pose.x, rough_pose.y, rough_pose.z, RAD2DEG(rough_pose.roll), RAD2DEG(rough_pose.pitch), RAD2DEG(rough_pose.yaw));

        bool bnb_success = true;
        auto bnb_opt_tmp = bnb_option;
        bnb_opt_tmp.min_score = 0.1;
        bnb_opt_tmp.linear_xy_window_size = 2;
        bnb_opt_tmp.linear_z_window_size = 0.5;
        bnb_opt_tmp.min_xy_resolution = 0.2;
        bnb_opt_tmp.min_z_resolution = 0.1;
        bnb_opt_tmp.angular_search_window = DEG2RAD(6);
        bnb_opt_tmp.min_angular_resolution = DEG2RAD(1);
        if (!bnb3d->MatchWithMatchOptions(rough_pose, rough_pose, scan, bnb_opt_tmp, lidar_ext))
        {
            bnb_success = false;
            LOG_ERROR("bnb_failed, when bnb min_score = %.2f!", bnb_opt_tmp.min_score);
        }
        if (bnb_success)
        {
            LOG_INFO("bnb_success!");
            LOG_WARN("bnb_pose = (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf), score_cnt = %d, time = %.2lf ms",
                     rough_pose.x, rough_pose.y, rough_pose.z, RAD2DEG(rough_pose.roll), RAD2DEG(rough_pose.pitch), RAD2DEG(rough_pose.yaw),
                     bnb3d->sort_cnt, timer.elapsedLast());
        }
    }
    else
    {
        LOG_ERROR("scan context failed, res index = %d, total descriptors = %lu! Please move the vehicle to another position and try again.", sc_res.first, trajectory_poses->size());
        return false;
    }
    return true;
}

bool Relocalization::run_manually_set(PointCloudType::Ptr scan, Eigen::Matrix4d &rough_mat, const Eigen::Matrix4d &lidar_ext)
{
    if (!prior_pose_inited)
    {
        LOG_WARN("wait for the boot position to be manually set!");
        return false;
    }

    Timer timer;
    bool bnb_success = true;
    if (!bnb3d->MatchWithMatchOptions(manual_pose, rough_pose, scan, bnb_option, lidar_ext))
    {
        auto bnb_opt_tmp = bnb_option;
        bnb_opt_tmp.min_score = 0.1;
        LOG_ERROR("bnb_failed, when bnb min_score = %.2f! min_score set to %.2f and try again.", bnb_option.min_score, bnb_opt_tmp.min_score);
        if (!bnb3d->MatchWithMatchOptions(manual_pose, rough_pose, scan, bnb_opt_tmp, lidar_ext))
        {
            bnb_success = false;
            rough_pose = manual_pose;
            LOG_ERROR("bnb_failed, when bnb min_score = %.2f!", bnb_opt_tmp.min_score);
        }
    }
    if (bnb_success)
    {
        LOG_INFO("bnb_success!");
        LOG_WARN("bnb_pose = (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf), score_cnt = %d, time = %.2lf ms",
                 rough_pose.x, rough_pose.y, rough_pose.z, RAD2DEG(rough_pose.roll), RAD2DEG(rough_pose.pitch), RAD2DEG(rough_pose.yaw),
                 bnb3d->sort_cnt, timer.elapsedLast());
    }
    return true;
}

bool Relocalization::run(const PointCloudType::Ptr &scan, Eigen::Matrix4d& result)
{
    Eigen::Matrix4d lidar_ext = lidar_extrinsic.toMatrix4d();
    bool success_flag = true;
    if (algorithm_type.compare("scan_context") == 0)
    {
        if (!run_scan_context(scan, result, lidar_ext) || !fine_tune_pose(scan, result, lidar_ext))
        {
#ifdef DEDUB_MODE
            result = EigenMath::CreateAffineMatrix(V3D(rough_pose.x, rough_pose.y, rough_pose.z), V3D(rough_pose.roll, rough_pose.pitch, rough_pose.yaw));
#endif
            success_flag = false;
        }
    }
    if (algorithm_type.compare("manually_set") == 0 || !success_flag)
    {
        success_flag = true;
        if (!run_manually_set(scan, result, lidar_ext) || !fine_tune_pose(scan, result, lidar_ext))
        {
#ifdef DEDUB_MODE
            result = EigenMath::CreateAffineMatrix(V3D(manual_pose.x, manual_pose.y, manual_pose.z), V3D(manual_pose.roll, manual_pose.pitch, manual_pose.yaw));
#endif
            success_flag = false;
        }
    }

    if (!success_flag)
    {
        LOG_ERROR("relocalization failed!");
        return false;
    }

    LOG_WARN("relocalization successfully!!!!!!");
    return true;
}

bool Relocalization::load_keyframe_descriptor(const std::string &path)
{
    if (!fs::exists(path))
        return false;

    int scd_file_count = 0, num_digits = 0;
    scd_file_count = FileOperation::getFilesNumByExtension(path, ".scd");

    if (scd_file_count != trajectory_poses->size())
        return false;

    num_digits = FileOperation::getOneFilenameByExtension(path, ".scd").length() - std::string(".scd").length();

    sc_manager->loadPriorSCD(path, num_digits, trajectory_poses->size());
    return true;
}

bool Relocalization::load_prior_map(const PointCloudType::Ptr& global_map)
{
    bnb3d = std::make_shared<BranchAndBoundMatcher3D>(global_map, bnb_option);

    voxel_filter.setLeafSize(gicp_downsample, gicp_downsample, gicp_downsample);
    voxel_filter.setInputCloud(global_map);
    voxel_filter.filter(*global_map);
    gicp.setInputTarget(global_map);
    return true;
}

bool Relocalization::plane_estimate(const pcl::PointCloud<PointType>::Ptr cluster, const float &threshold)
{
    MatrixXf pca_result(4, 1);
    MatrixXf A(cluster->points.size(), 3);
    MatrixXf b(cluster->points.size(), 1);
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < cluster->points.size(); j++)
    {
        A(j,0) = cluster->points[j].x;
        A(j,1) = cluster->points[j].y;
        A(j,2) = cluster->points[j].z;
    }

    V3F normvec = A.colPivHouseholderQr().solve(b);

    auto n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    int cnt = 0;
    for (int j = 0; j < cluster->points.size(); j++)
    {
        if (fabs(pca_result(0) * cluster->points[j].x + pca_result(1) * cluster->points[j].y + pca_result(2) * cluster->points[j].z + pca_result(3)) > threshold)
        {
            cnt++;
        }
    }
    auto percent = cnt * 1.0 / cluster->points.size();

    if (percent > plane_point_percent)
    {
        return false;
    }

    return true;
}

PointCloudType::Ptr Relocalization::plane_seg(PointCloudType::Ptr src)
{
    LOG_ERROR_COND(src->points.size() < 5000, "lidar point too little, size = %lu", src->points.size());

    PointCloudType::Ptr cloud(new PointCloudType());
    for (auto &p : *src)
    {
        if (pointDistanceSquare(p) > filter_radius * filter_radius)
        {
            cloud->push_back(p);
        }
    }

    PointCloudType::Ptr plane(new PointCloudType());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cluster_dis);
    ec.setMinClusterSize(min_plane_point);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int planeCount = 0;

    LOG_INFO("1 = %lu!", cluster_indices.size());
    for (const auto &cluster : cluster_indices)
    {
        LOG_INFO("1!");
        pcl::PointCloud<PointType>::Ptr cluster_cloud(new pcl::PointCloud<PointType>);
        pcl::ExtractIndices<PointType> extract;
        LOG_INFO("2!");
        extract.setInputCloud(cloud);
        extract.setIndices(boost::make_shared<pcl::PointIndices>(cluster));
        extract.setNegative(false);
        extract.filter(*cluster_cloud);
        LOG_INFO("3!");

        if (!plane_estimate(cluster_cloud, plane_dis))
        {
            continue;
        }
        LOG_INFO("4!");

        planeCount++;
        *plane += *cluster_cloud;
        LOG_INFO("5!");
    }
    LOG_INFO("6!");

    std::cout << "Plane extraction completed. Extracted " << planeCount << " planes. point num = " << plane->points.size() << std::endl;

    return plane;
}

bool Relocalization::fine_tune_pose(PointCloudType::Ptr scan, Eigen::Matrix4d &result, const Eigen::Matrix4d &lidar_ext)
{
    Timer timer;
    result = EigenMath::CreateAffineMatrix(V3D(rough_pose.x, rough_pose.y, rough_pose.z), V3D(rough_pose.roll, rough_pose.pitch, rough_pose.yaw));
    result *= lidar_ext; // imu pose -> lidar pose

#if 0
    auto scan = plane_seg(scan);
#endif

    PointCloudType::Ptr aligned(new PointCloudType());
    gicp.setMaxCorrespondenceDistance(search_radius);
    gicp.setMaximumIterations(150);
    gicp.setTransformationEpsilon(teps);
    gicp.setEuclideanFitnessEpsilon(feps);

    gicp.setInputSource(scan);
    gicp.align(*aligned, result.cast<float>());
    bool icp_rst = gicp.hasConverged();

    if (!icp_rst)
    {
        LOG_ERROR("GICP not converge!");
        return false;
    }
    else if (gicp.getFitnessScore() > fitness_score)
    {
        LOG_ERROR("failed! pointcloud registration fitness_score = %f.", gicp.getFitnessScore());
        return false;
    }
    if (gicp.getFitnessScore() < 0.1)
    {
        LOG_WARN("pointcloud registration fitness_score = %f.", gicp.getFitnessScore());
    }
    else
    {
        LOG_ERROR("pointcloud registration fitness_score = %f.", gicp.getFitnessScore());
    }

    result = gicp.getFinalTransformation().cast<double>();
    result *= lidar_ext.inverse();

    Eigen::Vector3d pos, euler;
    EigenMath::DecomposeAffineMatrix(result, pos, euler);
    LOG_WARN("gicp pose = (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf), gicp_time = %.2lf ms",
             pos(0), pos(1), pos(2), RAD2DEG(euler(0)), RAD2DEG(euler(1)), RAD2DEG(euler(2)), timer.elapsedLast());
    return true;
}

void Relocalization::set_init_pose(const Pose& _manual_pose)
{
    manual_pose = _manual_pose;
    LOG_WARN("*******************************************");
    LOG_WARN("set_init_pose = (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf)", manual_pose.x, manual_pose.y, manual_pose.z,
             RAD2DEG(manual_pose.roll), RAD2DEG(manual_pose.pitch), RAD2DEG(manual_pose.yaw));
    LOG_WARN("*******************************************");
    prior_pose_inited = true;
}

void Relocalization::set_bnb3d_param(const BnbOptions& match_option, const Pose& lidar_pose)
{
    bnb_option = match_option;
    LOG_WARN("*********** BnB Localizer Param ***********");
    LOG_WARN("linear_xy_window_size: %lf m", bnb_option.linear_xy_window_size);
    LOG_WARN("linear_z_window_size: %lf m", bnb_option.linear_z_window_size);
    LOG_WARN("angular_search_window: %lf degree", bnb_option.angular_search_window);
    std::stringstream resolutions;
    for (const auto& resolution: bnb_option.pc_resolutions)
    {
        resolutions << resolution << " ";
    }
    LOG_WARN("pc_resolutions: [ %s]", resolutions.str().c_str());
    LOG_WARN("bnb_depth: %d", bnb_option.bnb_depth);
    LOG_WARN("bnb_min_score: %lf", bnb_option.min_score);
    LOG_WARN("min_xy_resolution: %lf", bnb_option.min_xy_resolution);
    LOG_WARN("min_z_resolution: %lf", bnb_option.min_z_resolution);
    LOG_WARN("min_angular_resolution: %lf", bnb_option.min_angular_resolution);
    LOG_WARN("thread_num: %d", bnb_option.thread_num);
    LOG_WARN("filter_size_scan: %lf", bnb_option.filter_size_scan);
    LOG_WARN("debug_mode: %d", bnb_option.debug_mode);
    LOG_WARN("*******************************************");

    bnb_option.angular_search_window = DEG2RAD(bnb_option.angular_search_window);
    bnb_option.min_angular_resolution = DEG2RAD(bnb_option.min_angular_resolution);

    lidar_extrinsic = lidar_pose;
    LOG_WARN("*******************************************");
    LOG_WARN("lidar_ext = (%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf)", lidar_extrinsic.x, lidar_extrinsic.y, lidar_extrinsic.z,
             lidar_extrinsic.roll, lidar_extrinsic.pitch, lidar_extrinsic.yaw);
    LOG_WARN("*******************************************");
    lidar_extrinsic.roll = DEG2RAD(lidar_extrinsic.roll);
    lidar_extrinsic.pitch = DEG2RAD(lidar_extrinsic.pitch);
    lidar_extrinsic.yaw = DEG2RAD(lidar_extrinsic.yaw);
}

void Relocalization::set_plane_extract_param(const double &fr, const int &min_point, const double &clust_dis, const double &pl_dis, const double &point_percent)
{
    filter_radius = fr;
    min_plane_point = min_point;
    cluster_dis = clust_dis;
    plane_dis = pl_dis;
    plane_point_percent = point_percent;
}

void Relocalization::set_gicp_param(const double &gicp_ds, const double &search_radi, const double &tep, const double &fep, const double &fit_score)
{
    gicp_downsample = gicp_ds;
    search_radius = search_radi;
    teps = tep;
    feps = fep;
    fitness_score = fit_score;
}

void Relocalization::add_scancontext_descriptor(const PointCloudType::Ptr thiskeyframe, const std::string &path)
{
    PointCloudType::Ptr thiskeyframeDS(new PointCloudType());
    pcl::PointCloud<SCPointType>::Ptr sc_input(new pcl::PointCloud<SCPointType>());
    voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    voxel_filter.setInputCloud(thiskeyframe);
    voxel_filter.filter(*thiskeyframeDS);

    pcl::PointXYZI tmp;
    for (auto &point : thiskeyframeDS->points)
    {
        tmp.x = point.x;
        tmp.y = point.y;
        tmp.z = point.z;
        sc_input->push_back(tmp);
    }
    sc_manager->makeAndSaveScancontextAndKeys(*sc_input);

    if (path.compare("") != 0)
        sc_manager->saveCurrentSCD(path);
}
