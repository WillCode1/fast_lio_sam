#pragma once
#include <omp.h>
#include <math.h>
#include <thread>
#include "system/Header.h"
#include "FactorGraphOptimization.hpp"
#include "Relocalization.hpp"
#include "LoopClosure.hpp"

class MapStitch
{
public:
    MapStitch()
    {
        // keyframe_pose6d_optimized.reset(new pcl::PointCloud<PointXYZIRPYT>());
        // keyframe_scan.reset(new deque<PointCloudType::Ptr>());

        // backend = std::make_shared<FactorGraphOptimization>(keyframe_pose6d_optimized, keyframe_scan, gnss);
        // relocalization = make_shared<Relocalization>();
        // loopClosure = make_shared<LoopClosure>(relocalization->sc_manager);
    }

    void load_prior_map_info(const std::string& path)
    {

    }

    void load_stitch_map_info(const std::string& path)
    {
        
    }

};
