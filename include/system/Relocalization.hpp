#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include "global_localization/bnb3d.h"


class Relocalization
{
public:
    Relocalization();
    ~Relocalization();
    bool load_prior_map(const PointCloudType::Ptr &global_map);
    bool run(const PointCloudType::Ptr &scan, Eigen::Matrix4f &result);

    void set_init_pose(const Pose &_init_pose);
    void set_bnb3d_param(const BnbOptions &match_option, const Pose &init_pose, const Pose &lidar_pose);
    void set_plane_extract_param(const double &fr, const int &min_point, const double &clust_dis, const double &pl_dis, const double &point_percent);
    void set_gicp_param(const double &gicp_ds, const double &search_radi, const double &tep, const double &fep, const double &fit_score);

    bool need_wait_prior_pose_inited = true;
    BnbOptions bnb_option;
    Pose init_pose, lidar_pose, bnb_pose;
    std::shared_ptr<BranchAndBoundMatcher3D> bnb3d;

private:
    PointCloudType::Ptr plane_seg(PointCloudType::Ptr src);
    bool plane_estimate(const pcl::PointCloud<PointType>::Ptr cluster, const float &threshold);
    bool fine_tune_pose(PointCloudType::Ptr src, Eigen::Matrix4f &result);

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

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
};

Relocalization::Relocalization()
{
}

Relocalization::~Relocalization()
{
}

bool Relocalization::run(const PointCloudType::Ptr &scan, Eigen::Matrix4f& result)
{
    if (need_wait_prior_pose_inited && !prior_pose_inited)
        return false;

    // bnb3d
    Timer timer;
    Eigen::Matrix4d lidar_ext = lidar_pose.toMatrix4d();
    if (bnb_option.algorithm_type.compare("BNB_3D") == 0)
    {
        bool bnb_success = true;
        if (!bnb3d->MatchWithMatchOptions(init_pose, bnb_pose, scan, bnb_option, lidar_ext))
        {
            LOG_ERROR("bnb_failed, when bnb min_score = %.2f! min_score set to %.2f and try again.", bnb_option.min_score, 0.1);
            auto bnb_opt_tmp = bnb_option;
            bnb_opt_tmp.min_score = 0.1;
            if (!bnb3d->MatchWithMatchOptions(init_pose, bnb_pose, scan, bnb_opt_tmp, lidar_ext))
            {
                bnb_success = false;
                bnb_pose = init_pose;
                LOG_ERROR("bnb_failed, when bnb min_score = %.2f!", bnb_opt_tmp.min_score);
            }
        }
        if (bnb_success)
        {
            LOG_INFO("bnb_success!");
            LOG_WARN("bnb_pose = (%.2lf,%.2lf,%.4lf,%.0lf,%.0lf,%.2lf), score_cnt = %d, time = %.0lf ms",
                     bnb_pose.x, bnb_pose.y, bnb_pose.z, RAD2DEG(bnb_pose.roll), RAD2DEG(bnb_pose.pitch), RAD2DEG(bnb_pose.yaw),
                     bnb3d->sort_cnt, timer.elapsedLast());
        }
    }
    else
    {
        bnb_pose = init_pose;
    }

    // gicp match
    Eigen::Quaterniond bnb_quat = EigenRotation::RPY2Quaternion(V3D(bnb_pose.roll, bnb_pose.pitch, bnb_pose.yaw));
    result.setIdentity();
    result.topLeftCorner(3, 3) = bnb_quat.toRotationMatrix().cast<float>();
    result.topRightCorner(3, 1) = V3F(bnb_pose.x, bnb_pose.y, bnb_pose.z);
    timer.record();
#if 1
    result *= lidar_ext.cast<float>(); // add lidar extrinsic
    // auto plane_points = plane_seg(scan);
    if (!fine_tune_pose(scan, result))
    {
        return false;
    }
    result *= lidar_ext.inverse().cast<float>();
#endif
    Eigen::Vector3f pos = result.topRightCorner(3, 1);
    Eigen::Vector3f euler = EigenRotation::RotationMatrix2RPY2(M3D(result.topLeftCorner(3, 3).cast<double>())).cast<float>();
    LOG_WARN("gicp pose = (%f, %f, %f, %f, %f, %f), gicp_time = %.2lf ms",
             pos(0), pos(1), pos(2), RAD2DEG(euler(0)), RAD2DEG(euler(1)), RAD2DEG(euler(2)), timer.elapsedLast());

    LOG_WARN("relocalization successfully!!!!!!");
    return true;
}

bool Relocalization::load_prior_map(const PointCloudType::Ptr& global_map)
{
    bnb3d = std::make_shared<BranchAndBoundMatcher3D>(global_map, bnb_option);

    pcl::VoxelGrid<PointType> voxel_filter;
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

    // if (pub_plane_point)
    // {
    //     int size = plane->points.size();
    //     PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));

    //     for (int i = 0; i < size; i++)
    //     {
    //         RGBpointBodyToWorld(&plane->points[i], &laserCloudWorld->points[i]);
    //     }

    //     sensor_msgs::PointCloud2 map_msg;
    //     pcl::toROSMsg(*laserCloudWorld, map_msg);
    //     map_msg.header.stamp = ros::Time::now();
    //     map_msg.header.frame_id = "camera_init";
    //     pubPlaneSeg.publish(map_msg);
    // }

    return plane;
}

bool Relocalization::fine_tune_pose(PointCloudType::Ptr src, Eigen::Matrix4f& result)
{
    PointCloudType::Ptr aligned(new PointCloudType());
    gicp.setMaxCorrespondenceDistance(search_radius);
    gicp.setMaximumIterations(150);
    gicp.setTransformationEpsilon(teps);
    gicp.setEuclideanFitnessEpsilon(feps);

    gicp.setInputSource(src);
    gicp.align(*aligned, result);
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

    result = gicp.getFinalTransformation();
    return true;
}

void Relocalization::set_init_pose(const Pose& _init_pose)
{
    init_pose = _init_pose;
    LOG_WARN("*******************************************");
    LOG_WARN("set_init_pose = (%.2lf,%.2lf,%.4lf,%.0lf,%.0lf,%.2lf)", init_pose.x, init_pose.y, init_pose.z,
             init_pose.roll, init_pose.pitch, init_pose.yaw);
    LOG_WARN("*******************************************");
    init_pose.roll = DEG2RAD(init_pose.roll);
    init_pose.pitch = DEG2RAD(init_pose.pitch);
    init_pose.yaw = DEG2RAD(init_pose.yaw);
    prior_pose_inited = true;
}

void Relocalization::set_bnb3d_param(const BnbOptions& match_option, const Pose& _init_pose, const Pose& _lidar_pose)
{
    bnb_option = match_option;
    LOG_WARN("*********** BnB Localizer Param ***********");
    LOG_WARN("algorithm_type: %s", bnb_option.algorithm_type.c_str());
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

    init_pose = _init_pose;
    lidar_pose = _lidar_pose;
    LOG_WARN("*******************************************");
    LOG_WARN("init_pose = (%.2lf,%.2lf,%.4lf,%.0lf,%.0lf,%.2lf)", init_pose.x, init_pose.y, init_pose.z,
             init_pose.roll, init_pose.pitch, init_pose.yaw);
    LOG_WARN("lidar_ext = (%.2lf,%.2lf,%.4lf,%.0lf,%.0lf,%.2lf)", lidar_pose.x, lidar_pose.y, lidar_pose.z,
             lidar_pose.roll, lidar_pose.pitch, lidar_pose.yaw);
    LOG_WARN("*******************************************");
    init_pose.roll = DEG2RAD(init_pose.roll);
    init_pose.pitch = DEG2RAD(init_pose.pitch);
    init_pose.yaw = DEG2RAD(init_pose.yaw);
    lidar_pose.roll = DEG2RAD(lidar_pose.roll);
    lidar_pose.pitch = DEG2RAD(lidar_pose.pitch);
    lidar_pose.yaw = DEG2RAD(lidar_pose.yaw);
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
