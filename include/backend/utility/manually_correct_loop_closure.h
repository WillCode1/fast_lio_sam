#pragma once
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pangolin/pangolin.h>
#include "../Header.h"

class ManuallyCorrectLoopClosure
{
public:
    double getFitnessScore(PointCloudType::Ptr submap, PointCloudType::Ptr keyframe, const Eigen::Affine3f &trans, double max_range)
    {
        double fitness_score = 0.0;

        // Transform the input dataset using the final transformation
        PointCloudType::Ptr scan_tmp(new PointCloudType);
        pcl::transformPointCloud(*keyframe, *scan_tmp, trans);

        pcl::search::KdTree<PointType> kdtree;
        kdtree.setInputCloud(submap);

        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);

        // For each point in the source dataset
        int nr = 0;
        for (size_t i = 0; i < scan_tmp->points.size(); ++i)
        {
            // Find its nearest neighbor in the target
            kdtree.nearestKSearch(scan_tmp->points[i], 1, nn_indices, nn_dists);

            // Deal with occlusions (incomplete targets)
            if (nn_dists[0] <= max_range)
            {
                // Add to the fitness score
                fitness_score += nn_dists[0];
                nr++;
            }
        }

        if (nr > 0)
            return (fitness_score / nr);
        else
            return (std::numeric_limits<double>::max());
    }

    double manually_adjust_loop_closure(PointCloudType::Ptr submap, PointCloudType::Ptr keyframe, pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d,
                                        const Eigen::Affine3f &posetransform, Eigen::Affine3f &tuningLidarFrame, bool &reject_this_loop)
    {
        // 计算点云边界
        float min_x = std::numeric_limits<float>::max(), max_x = -min_x;
        float min_y = min_x, max_y = -min_x, min_z = min_x, max_z = -min_x;
        for (const auto &point : keyframe->points)
        {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }
        Eigen::Vector3f center = posetransform.translation();
        float size = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
        float distance = 40;

        // 创建Pangolin窗口
        const int window_width = 1920;
        const int window_height = 1080;
        pangolin::CreateWindowAndBind("Loop Closure Tuning", window_width, window_height);
        glEnable(GL_DEPTH_TEST);

        Eigen::Vector3f eye(center.x(), center.y(), center.z() + distance);
        Eigen::Vector3f look = (center - eye).normalized();
        Eigen::Vector3f up(0, 1, 0);

        // 计算3D视图中心
        const float ui_width = 180.0f; // UI面板固定宽度
        float view_width = window_width - ui_width;
        float u0 = ui_width + view_width / 2;
        float v0 = window_height / 2;

        // 设置相机
        pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(window_width, window_height, 420, 420, u0, v0, 0.1, size * 10),
            pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                      center.x(), center.y(), center.z(),
                                      up.x(), up.y(), up.z()));

        // 创建3D显示区域
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, ui_width / window_width, 1.0)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

        // 创建UI面板
        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, ui_width / window_width);

        // 添加滑块
        pangolin::Var<float> trans_x("ui.X Trans", 0.0f, -5.0f, 5.0f);
        pangolin::Var<float> trans_y("ui.Y Trans", 0.0f, -5.0f, 5.0f);
        pangolin::Var<float> trans_z("ui.Z Trans", 0.0f, -1.0f, 1.0f);
        pangolin::Var<float> rot_x("ui.Roll (rad)", 0.0f, RAD2DEG(-0.2f), RAD2DEG(0.2f));
        pangolin::Var<float> rot_y("ui.Pitch (rad)", 0.0f, RAD2DEG(-0.2f), RAD2DEG(0.2f));
        pangolin::Var<float> rot_z("ui.Yaw (rad)", 0.0f, RAD2DEG(-3.14f), RAD2DEG(3.14f));
        pangolin::Var<bool> reset("ui.Reset", false, false);
        pangolin::Var<bool> screenshot("ui.Screen Shot", false, false);
        pangolin::Var<bool> reject_loop("ui.Reject Loop", false, false);
        pangolin::Var<bool> confirm_loop("ui.Confirm Loop", false, false);
        pangolin::Var<float> fitness_score("ui.Score", 1.0f);
        pangolin::Var<bool> view_top("ui.View Top", false, false);
        pangolin::Var<bool> view_bottom("ui.View Bottom", false, false);
        pangolin::Var<bool> view_front("ui.View Front", false, false);
        pangolin::Var<bool> view_back("ui.View Back", false, false);
        pangolin::Var<bool> view_left("ui.View Left", false, false);
        pangolin::Var<bool> view_right("ui.View Right", false, false);

        bool first_run = true;
        while (!pangolin::ShouldQuit())
        {
            // 清除屏幕
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (first_run || reset.GuiChanged())
            {
                trans_x = 0.0f;
                trans_y = 0.0f;
                trans_z = 0.0f;
                rot_x = 0.0f;
                rot_y = 0.0f;
                rot_z = 0.0f;
                reset = false;
                first_run = false;
            }

#if 0
            // 检测截屏按钮
            if (screenshot.GuiChanged())
            {
                std::time_t now = std::time(nullptr);
                std::stringstream ss;
                ss << "./screenshot_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") << ".png";
                d_cam.SaveRenderNow(ss.str());
                std::cout << "截屏已保存为 " << ss.str() << std::endl;
            }
#endif

            if (confirm_loop.GuiChanged())
            {
                pangolin::Quit();
                break;
            }

            if (reject_loop.GuiChanged())
            {
                reject_this_loop = true;
                pangolin::Quit();
            }

            // 检测视图切换
            if (view_front.GuiChanged())
            {
                Eigen::Vector3f eye(center.x(), center.y() - distance, center.z());
                Eigen::Vector3f up(0, 0, 1);
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                                                   center.x(), center.y(), center.z(),
                                                                   up.x(), up.y(), up.z()));
            }
            if (view_back.GuiChanged())
            {
                Eigen::Vector3f eye(center.x(), center.y() + distance, center.z());
                Eigen::Vector3f up(0, 0, 1);
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                                                   center.x(), center.y(), center.z(),
                                                                   up.x(), up.y(), up.z()));
            }
            if (view_left.GuiChanged())
            {
                Eigen::Vector3f eye(center.x() - distance, center.y(), center.z());
                Eigen::Vector3f up(0, 0, 1);
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                                                   center.x(), center.y(), center.z(),
                                                                   up.x(), up.y(), up.z()));
            }
            if (view_right.GuiChanged())
            {
                Eigen::Vector3f eye(center.x() + distance, center.y(), center.z());
                Eigen::Vector3f up(0, 0, 1);
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                                                   center.x(), center.y(), center.z(),
                                                                   up.x(), up.y(), up.z()));
            }
            if (view_top.GuiChanged())
            {
                Eigen::Vector3f eye(center.x(), center.y(), center.z() + distance);
                Eigen::Vector3f up(0, 1, 0);
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                                                   center.x(), center.y(), center.z(),
                                                                   up.x(), up.y(), up.z()));
            }
            if (view_bottom.GuiChanged())
            {
                Eigen::Vector3f eye(center.x(), center.y(), center.z() - distance);
                Eigen::Vector3f up(0, -1, 0);
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(eye.x(), eye.y(), eye.z(),
                                                                   center.x(), center.y(), center.z(),
                                                                   up.x(), up.y(), up.z()));
            }

            // 激活3D显示
            d_cam.Activate(s_cam);

            // 绘制子地图（红色）
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_POINTS);
            for (const auto &point : submap->points)
            {
                glVertex3f(point.x, point.y, point.z);
            }
            glEnd();

            // 计算变换
            tuningLidarFrame.setIdentity();
            tuningLidarFrame.translation() = Eigen::Vector3f(trans_x, trans_y, trans_z);
            tuningLidarFrame.rotate(Eigen::AngleAxisf(DEG2RAD(rot_z), Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(DEG2RAD(rot_y), Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(DEG2RAD(rot_x), Eigen::Vector3f::UnitX()));

            PointCloudType::Ptr transformed_scan(new PointCloudType);
            pcl::transformPointCloud(*keyframe, *transformed_scan, posetransform * tuningLidarFrame);

            // 绘制变换后的扫描点云（绿色）
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_POINTS);
            for (const auto &point : transformed_scan->points)
            {
                glVertex3f(point.x, point.y, point.z);
            }
            glEnd();

            // 绘制轨迹线
            glLineWidth(3);
            glColor3f(1.0, 1.0, 0.0); // 黄色轨迹
            glBegin(GL_LINE_STRIP);
            for (auto i = 0; i < keyframe_pose6d->points.size() - 1; ++i)
            {
                glVertex3f(keyframe_pose6d->points[i].x, keyframe_pose6d->points[i].y, keyframe_pose6d->points[i].z);
            }
            glColor3f(0.0, 0.0, 1.0); // 黄色轨迹
            glVertex3f(keyframe_pose6d->points[keyframe_pose6d->points.size() - 1].x,
                       keyframe_pose6d->points[keyframe_pose6d->points.size() - 1].y,
                       keyframe_pose6d->points[keyframe_pose6d->points.size() - 1].z);
            glVertex3f(center.x(), center.y(), center.z());
            glEnd();

            fitness_score = getFitnessScore(submap, keyframe, posetransform * tuningLidarFrame, 2);

            // 完成帧渲染
            pangolin::FinishFrame();
        }

        pangolin::DestroyWindow("Loop Closure Tuning");
        pcl::getTransformation(trans_x, trans_y, trans_z, DEG2RAD(rot_x), DEG2RAD(rot_y), DEG2RAD(rot_z), tuningLidarFrame);
        return fitness_score;
    }
};
