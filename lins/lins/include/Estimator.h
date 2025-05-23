/* done
Description:
Author     : Ji Qingshi
date       :
*/

#ifndef INCLUDE_ESTIMATOR_H_
#define INCLUDE_ESTIMATOR_H_

#include <MapRingBuffer.h>
#include <math_utils.h>
#include <nav_msgs/Odometry.h>
#include <parameters.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tic_toc.h>

#include <StateEstimator.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <sensor_utils.hpp>

#include "cloud_msgs/cloud_info.h"

using namespace std;
using namespace Eigen;
using namespace math_utils;
using namespace sensor_utils;

namespace fusion
{
class LinsFusion
{
 public:
  LinsFusion(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~LinsFusion();

  void run();
  void initialization();
  void publishTopics(); // publish corner surf outlier and tf
  void publishOdometryYZX(double timeStamp);
  inline void publishCloudMsg(ros::Publisher &publisher,
                              pcl::PointCloud<PointType>::Ptr cloud,
                              const ros::Time &stamp,
                              const std::string &frameID)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = stamp;
    msg.header.frame_id = frameID; // coordinate ID
    publisher.publish(msg);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &imuIn);
  void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
  void laserCloudInfoCallback(const cloud_msgs::cloud_infoConstPtr &msgIn);
  void outlierCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
  void mapOdometryCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg);

  void performStateEstimation();
  void processFirstPointCloud();
  bool processPointClouds();
  void performImuBiasEstimation();
  void alignIMUtoVehicle(const V3D &rpy, const V3D &acc_in, const V3D &gyr_in, V3D &acc_out, V3D &gyr_out);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // StateEstimator
  StateEstimator *estimator;

  // Subscriber
  ros::Subscriber subLaserCloud;
  ros::Subscriber subLaserCloudInfo;
  ros::Subscriber subOutlierCloud;
  ros::Subscriber subImu;
  ros::Subscriber subGPS_;
  ros::Subscriber subMapOdom_;

  // Publishers
  ros::Publisher pubUndistortedPointCloud;
  // feature
  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;
  // feature from last scan
  ros::Publisher pubLaserCloudCornerLast;
  ros::Publisher pubLaserCloudSurfLast;
  ros::Publisher pubOutlierCloudLast;
  // odometry
  ros::Publisher pubLaserOdometry;
  ros::Publisher pubIMUOdometry;
  ros::Publisher pubGpsOdometry;
  ros::Publisher pubLaserOdom;

  // PointCloudPtrs
  pcl::PointCloud<PointType>::Ptr distortedPointCloud;
  pcl::PointCloud<PointType>::Ptr outlierPointCloud;

  // Messages
  nav_msgs::Odometry laserOdom;
  nav_msgs::Odometry laserOdometry;
  nav_msgs::Odometry imuOdometry;
  nav_msgs::Odometry gpsOdometry;

  // Buffers
  MapRingBuffer<Imu> imuBuf_;
  MapRingBuffer<sensor_msgs::PointCloud2::ConstPtr> pclBuf_;
  MapRingBuffer<sensor_msgs::PointCloud2::ConstPtr> outlierBuf_;
  MapRingBuffer<cloud_msgs::cloud_info> cloudInfoBuf_;
  MapRingBuffer<Gps> gpsBuf_;

  // Time
  int scan_counter_; // how many times of "process Point Clouds"
  double duration_;

  // Measurements
  V3D acc_raw_;
  V3D gyr_raw_;
  V3D acc_aligned_;
  V3D gyr_aligned_;
  V3D misalign_euler_angles_;
  V3D ba_tmp_;
  V3D bw_tmp_;
  V3D ba_init_; // 没有使用
  V3D bw_init_; // 没有使用
  double scan_time_;
  double last_imu_time_;
  double last_scan_time_;
  int sample_counter_;
  bool isInititialized;
  bool isImuCalibrated;
};
} // namespace fusion

#endif // INCLUDE_ESTIMATOR_H_