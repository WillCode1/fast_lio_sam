#pragma once
#include <deque>
#include "utility/Header.h"

struct GnssPose
{
  GnssPose(const double &time = 0, const V3D &pos = ZERO3D, const V3D &cov = ZERO3D)
      : timestamp(time), position(pos), covariance(cov) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  V3D position; // longitude, latitude, altitude
  V3D covariance;
};

class GnssProcessor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void set_extrinsic(const V3D &transl = ZERO3D, const M3D &rot = EYE3D);
  void gnss_handler(const GnssPose &gps_raw);
  bool get_gnss_factor(GnssPose &thisGPS, const double &lidar_end_time, const double &odom_z);

  float gpsCovThreshold;
  bool useGpsElevation = false; //  是否使用gps高层优化
  deque<GnssPose> gnss_buffer;

private:
  M3D ext_R_Gnss_Imu;
  V3D ext_T_Gnss_Imu;
};

void GnssProcessor::set_extrinsic(const V3D &transl, const M3D &rot)
{
  ext_T_Gnss_Imu = transl;
  ext_R_Gnss_Imu = rot;
}

void GnssProcessor::gnss_handler(const GnssPose &gps_raw)
{
  static int count = 0;
  static GnssPose gnss_origin;
  static const double R_EARTH = 6371393.0; // m
  double deg2rad = M_PI / 180.0;
  count++;

  if (count < 10)
  {
    gnss_origin = gps_raw;
    printf("--gnss_origin: lon: %.7f, lat: %.7f, alt: %.3f \n", gnss_origin.position.x(), gnss_origin.position.y(), gnss_origin.position.z());
    return;
  }
  const double &d_lon = gps_raw.position.x() - gnss_origin.position.x();
  const double &d_lat = gps_raw.position.y() - gnss_origin.position.y();
  const double &d_alt = gps_raw.position.z() - gnss_origin.position.z();

  V3D enu;
  enu.x() = d_lon * R_EARTH * cos(gps_raw.position.y() * deg2rad) * deg2rad;
  enu.y() = d_lat * R_EARTH * deg2rad;
  enu.z() = d_alt;
  GnssPose res = gps_raw;
  res.position = ext_R_Gnss_Imu * enu + ext_T_Gnss_Imu; // frame gnss -> imu

  gnss_buffer.push_back(res);
}

bool GnssProcessor::get_gnss_factor(GnssPose &thisGPS, const double &lidar_end_time, const double &odom_z)
{
  static PointType lastGPSPoint;
  while (!gnss_buffer.empty())
  {
    if (gnss_buffer.front().timestamp < lidar_end_time - 0.05)
    {
      gnss_buffer.pop_front();
    }
    else if (gnss_buffer.front().timestamp > lidar_end_time + 0.05)
    {
      return false;
    }
    else
    {
      thisGPS = gnss_buffer.front();
      gnss_buffer.pop_front();

      // GPS噪声协方差太大，不能用
      float noise_x = thisGPS.covariance(0);
      float noise_y = thisGPS.covariance(1);
      if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
        continue;

      float gps_x = thisGPS.position(0);
      float gps_y = thisGPS.position(1);
      float gps_z = thisGPS.position(2);
      if (!useGpsElevation)
      {
        thisGPS.position(2) = odom_z;
        thisGPS.covariance(2) = 0.01;
      }

      // (0,0,0)无效数据
      if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
        continue;

      PointType curGPSPoint;
      curGPSPoint.x = gps_x;
      curGPSPoint.y = gps_y;
      curGPSPoint.z = gps_z;

      if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
        continue;
      else
        lastGPSPoint = curGPSPoint;
      return true;
    }
  }
  return false;
}
