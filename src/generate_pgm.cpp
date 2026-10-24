#include "backend/Header.h"
#include "backend/utility/Pcd2Pgm.h"
#include <iomanip>
FILE *location_log = nullptr;

// 读取关键帧原始点云（雷达/机体原始坐标系）。此处不再做高度切片，
// 去倾斜与高度切片统一在 pointcloudKeyframeToWorldSliced() 中完成。
void load_keyframe(const std::string &keyframe_path, PointCloudType::Ptr keyframe_pc,
                   int keyframe_cnt, int num_digits = 6)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << keyframe_cnt;
    std::string keyframe_idx = out.str();
    string keyframe_file(keyframe_path + keyframe_idx + string(".pcd"));
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(keyframe_file, *tmp_pc);
    for (auto i = 0; i < tmp_pc->points.size(); ++i)
    {
        if (tmp_pc->points[i].x < 0) // 保留原行为：丢弃雷达系后方(x<0)的点
            continue;
        PointType point;
        pcl::copyPoint(tmp_pc->points[i], point);
        keyframe_pc->points.emplace_back(point);
    }
}

// min_z/max_z: 去倾斜后、相对传感器原点的垂向高度切片范围（此时 z 轴已竖直）
void save_pgm(const double &pgm_resolution = 0.05, const float &min_z = -1.5, const float &max_z = 0.1)
{
    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d(new pcl::PointCloud<PointXYZIRPYT>());
    PointCloudType::Ptr global_map(new PointCloudType());
    pcl::io::loadPCDFile(PCD_FILE_DIR("trajectory.pcd"), *keyframe_pose6d);
    for (auto i = 0; i < keyframe_pose6d->size(); ++i)
    {
        PointCloudType::Ptr keyframe_pc(new PointCloudType());
        load_keyframe(PCD_FILE_DIR("keyframe/"), keyframe_pc, i, 6);
        // octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
        // 先按该帧位姿把机体轴旋转到与世界系平行（z 轴竖直，去倾斜），再按相对传感器高度切片，最后平移到世界系
        *global_map += *pointcloudKeyframeToWorldSliced(keyframe_pc, (*keyframe_pose6d)[i], min_z, max_z);
    }
    Pcd2Pgm mg(pgm_resolution, PCD_FILE_DIR("") + "/map");
    mg.convert_from_pcd(global_map);
    mg.convert_to_pgm();
    LOG_WARN("Success save pgm to %s.", PCD_FILE_DIR("").c_str());
}

int main(int argc, char **argv)
{
    string param1, param2, param3;
    if (argc == 2 || argc == 3 || argc == 4)
        param1 = argv[1];
    if (argc == 3 || argc == 4)
        param2 = argv[2];
    if (argc == 4)
        param3 = argv[3];

    if (argc == 2)
        save_pgm(std::stod(param1));
    else if (argc == 3)
        save_pgm(std::stod(param1), std::stod(param2));
    else if (argc == 4)
        save_pgm(std::stod(param1), std::stod(param2), std::stod(param3));
    else
        save_pgm();

    return 0;
}