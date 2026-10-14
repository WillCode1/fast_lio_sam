/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * @ref: https://github.com/ros-planning/navigation/tree/kinetic-devel/map_server/src
 */
#pragma once
#include <cstdio>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;

/**
 * @brief Map generation node.
 */
class Pcd2Pgm
{
public:
  Pcd2Pgm(const double &map_resolution, const std::string &save_path);

  void convert_from_pcd(const PointCloudType::Ptr cloud);

  void convert_to_pgm();

  void convert_from_pgm();

  double resolution_;
  double offset_yaw_{0};
  std::string save_path_;

  uint32_t map_info_width;
  uint32_t map_info_height;
  double map_info_origin_position_x;
  double map_info_origin_position_y;
  std::vector<int8_t> map_data;
};
