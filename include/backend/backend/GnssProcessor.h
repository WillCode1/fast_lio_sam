#pragma once
#include <deque>
#include "backend/Header.h"
#include "backend/global_localization/UtmCoordinate.h"
#include "backend/global_localization/EnuCoordinate.h"
// #define ENU

struct GnssPose
{
  GnssPose(const double &time = 0, const V3D &pos = ZERO3D, const QD &rot = EYEQD, const Eigen::VectorXd &cov = Eigen::VectorXd::Zero(6));

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  V3D gnss_position;
  QD gnss_quat;
  Eigen::VectorXd covariance;

  float current_gnss_interval;
  V3D lidar_pos_fix; // utm add extrinsic
};

/*
 * Adding gnss factors to the mapping requires a lower speed
 */
class GnssProcessor
{
public:
  GnssProcessor();

  ~GnssProcessor();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void set_extrinsic(const V3D &transl = ZERO3D, const M3D &rot = EYE3D);
  V3D gnss_global2local(const V3D &gnss_position);
  void gnss_handler(const GnssPose &gnss_raw);
  bool get_gnss_factor(GnssPose &thisGPS, const double &lidar_end_time, const double &odom_z);

  int numsv = 20;
  float rtk_age = 30;
  vector<double> gpsCovThreshold;

  float gnssValidInterval = 0.2;
  bool useGpsElevation = false;
  deque<GnssPose> gnss_buffer;
  Eigen::Matrix4d extrinsic_lidar2gnss;

private:
#ifndef NO_LOGER
  FILE *file_pose_gnss;
#endif
};
