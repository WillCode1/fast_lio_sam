#include "backend/Header.h"
FILE *location_log = nullptr;

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
        PointType point;
        pcl::copyPoint(tmp_pc->points[i], point);
        keyframe_pc->points.emplace_back(point);
    }
}

int main()
{
    string trajectory_path = PCD_FILE_DIR("trajectory.pcd");
    string keyframe_path = PCD_FILE_DIR("keyframe/");
    string globalmap_path = PCD_FILE_DIR("globalmap.pcd");
    string globalmap_raw_path = PCD_FILE_DIR("globalmap_raw.pcd");

    pcl::PointCloud<PointXYZIRPYT>::Ptr keyframe_pose6d(new pcl::PointCloud<PointXYZIRPYT>());
    PointCloudType::Ptr global_map(new PointCloudType());
    pcl::io::loadPCDFile(trajectory_path, *keyframe_pose6d);
    for (auto i = 0; i < keyframe_pose6d->size(); ++i)
    {
        PointCloudType::Ptr keyframe_pc(new PointCloudType());
        load_keyframe(keyframe_path, keyframe_pc, i, 6);
        // octreeDownsampling(keyframe_pc, keyframe_pc, 0.1);
        *global_map += *pointcloudKeyframeToWorld(keyframe_pc, (*keyframe_pose6d)[i]);
    }
    savePCDFile(globalmap_raw_path, *global_map);
    octreeDownsampling(global_map, global_map, 0.2);
    savePCDFile(globalmap_path, *global_map);
    LOG_WARN("Success save global map to %s.", globalmap_path.c_str());
    return 0;
}
