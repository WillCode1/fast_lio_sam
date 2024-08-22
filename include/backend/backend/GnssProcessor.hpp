#pragma once
#include <deque>
#include "backend/Header.h"
#include "backend/global_localization/UtmCoordinate.h"
#include "backend/global_localization/EnuCoordinate.h"
#define ENU

struct GnssPose
{
  GnssPose(const double &time = 0, const V3D &pos = ZERO3D, const QD &rot = EYEQD, const V3D &cov = ZERO3D)
      : timestamp(time), gnss_position(pos), gnss_quat(rot.normalized()), covariance(cov) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  V3D gnss_position;
  QD gnss_quat;
  V3D covariance;

  float current_gnss_interval;
  V3D lidar_pos_fix; // utm add extrinsic
};

/*
 * Adding gnss factors to the mapping requires a lower speed
 */
class GnssProcessor
{
public:
  GnssProcessor()
  {
    extrinsic_lidar2gnss.setIdentity();
#ifndef NO_LOGER
    file_pose_gnss = fopen(DEBUG_FILE_DIR("gnss_pose.txt").c_str(), "w");
    fprintf(file_pose_gnss, "# gnss trajectory\n# timestamp tx ty tz qx qy qz qw\n");
#endif
  }

  ~GnssProcessor()
  {
#ifndef NO_LOGER
    fclose(file_pose_gnss);
#endif
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void set_extrinsic(const V3D &transl = ZERO3D, const M3D &rot = EYE3D);
  void gnss_handler(const GnssPose &gnss_raw);
  bool get_gnss_factor(GnssPose &thisGPS, const double &lidar_end_time, const double &odom_z);

  float gnssValidInterval = 0.2;
  float gpsCovThreshold = 2;
  bool useGpsElevation = false;
  deque<GnssPose> gnss_buffer;
  Eigen::Matrix4d extrinsic_lidar2gnss;

private:
  bool check_mean_and_variance(const std::vector<V3D> &start_point, utm_coordinate::utm_point &utm_origin, const double &variance_thold);

private:
#ifndef NO_LOGER
  FILE *file_pose_gnss;
#endif
};

void GnssProcessor::set_extrinsic(const V3D &transl, const M3D &rot)
{
  extrinsic_lidar2gnss.setIdentity();
  extrinsic_lidar2gnss.topLeftCorner(3, 3) = rot;
  extrinsic_lidar2gnss.topRightCorner(3, 1) = transl;
}

bool GnssProcessor::check_mean_and_variance(const std::vector<V3D> &start_point, utm_coordinate::utm_point &utm_origin, const double &variance_thold)
{
  V3D mean = V3D::Zero();
  V3D variance = V3D::Zero();

  for (const V3D &vec : start_point)
  {
    mean += vec;
  }
  mean /= start_point.size();

  for (const V3D &vec : start_point)
  {
    V3D diff = vec - mean;
    variance.x() += diff.x() * diff.x();
    variance.y() += diff.y() * diff.y();
    variance.z() += diff.z() * diff.z();
  }
  variance /= (start_point.size() - 1); // 使用样本方差，除以 (n-1)

  LOG_WARN("check_mean_and_variance. mean = (%.5f, %.5f, %.5f), variance = (%.5f, %.5f, %.5f).", mean.x(), mean.y(), mean.z(), variance.x(), variance.y(), variance.z());

  if (variance.x() > variance_thold || variance.y() > variance_thold || variance.z() > variance_thold)
    return false;

  utm_origin.east = mean.x();
  utm_origin.north = mean.y();
  utm_origin.up = mean.z();
  return true;
}

void GnssProcessor::gnss_handler(const GnssPose &gnss_raw)
{
  GnssPose gps_pose = gnss_raw;
#ifdef ENU
  gps_pose.gnss_position = enu_coordinate::Earth::LLH2ENU(gnss_raw.gnss_position, true);
#else
  gps_pose.gnss_position = utm_coordinate::LLAtoUTM2(gnss_raw.gnss_position);
#endif
  gnss_buffer.push_back(gps_pose);
#ifndef NO_LOGER
  LogAnalysis::save_trajectory(file_pose_gnss, gps_pose.gnss_position, gps_pose.gnss_quat, gps_pose.timestamp);
#endif
}

bool GnssProcessor::get_gnss_factor(GnssPose &thisGPS, const double &lidar_end_time, const double &odom_z)
{
  while (!gnss_buffer.empty())
  {
    const auto &header_msg = gnss_buffer.front();
    if (header_msg.covariance(0) > gpsCovThreshold || header_msg.covariance(1) > gpsCovThreshold)
    {
      LOG_WARN("GPS noise covariance is too large (%f, %f), threshold = %f, ignored!",
               header_msg.covariance(0), header_msg.covariance(1), gpsCovThreshold);
      gnss_buffer.pop_front();
    }
    else if (header_msg.timestamp < lidar_end_time - gnssValidInterval)
    {
      gnss_buffer.pop_front();
    }
    else if (header_msg.timestamp > lidar_end_time + gnssValidInterval)
    {
      return false;
    }
    else
    {
      // find the one with the smallest time interval.
      thisGPS.current_gnss_interval = gnssValidInterval;
      while (!gnss_buffer.empty())
      {
        auto current_gnss_interval = std::abs(gnss_buffer.front().timestamp - lidar_end_time);
        if (current_gnss_interval < thisGPS.current_gnss_interval)
        {
          thisGPS = gnss_buffer.front();
          thisGPS.current_gnss_interval = current_gnss_interval;
          gnss_buffer.pop_front();
        }
        else
          break;
      }

      Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
      gnss_pose.topLeftCorner(3, 3) = thisGPS.gnss_quat.toRotationMatrix();
      gnss_pose.topRightCorner(3, 1) = thisGPS.gnss_position;
      gnss_pose *= extrinsic_lidar2gnss;
      thisGPS.lidar_pos_fix = gnss_pose.topRightCorner(3, 1);

      if (!useGpsElevation)
      {
        thisGPS.lidar_pos_fix(2) = odom_z;
        thisGPS.covariance(2) = 0.01;
      }
      return true;
    }
  }
  return false;
}
