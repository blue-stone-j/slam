/*
Description:
Author     : Ji Qingshi
date       :
*/

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "lidar.h"
#include "laserProcessingClass.h"

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf; // save point cloud
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  mutex_lock.lock( );
  pointCloudBuf.push(laserCloudMsg);
  mutex_lock.unlock( );
}

double total_time = 0; // time of extracting from all frames/clouds
int total_frame   = 0; // num of all frames

void laser_processing( )
{
  while (1)
  {
    if (!pointCloudBuf.empty( ))
    {
      // read data
      mutex_lock.lock( );
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>( ));
      pcl::fromROSMsg(*pointCloudBuf.front( ), *pointcloud_in);
      ros::Time pointcloud_time = (pointCloudBuf.front( ))->header.stamp;
      pointCloudBuf.pop( );
      mutex_lock.unlock( );
      // get first element in queue

      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>( ));
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>( ));

      std::chrono::time_point<std::chrono::system_clock> start, end; // record time of feature extraction
      start = std::chrono::system_clock::now( );
      // extract surface and corner features
      laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
      end                                          = std::chrono::system_clock::now( );
      std::chrono::duration<float> elapsed_seconds = end - start; // unit is second
      total_frame++;
      float time_temp = elapsed_seconds.count( ) * 1000; // unit is milliseconds
      total_time += time_temp;
      // ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

      // publish all features: surface+corner
      sensor_msgs::PointCloud2 laserCloudFilteredMsg;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>( ));
      *pointcloud_filtered += *pointcloud_edge;
      *pointcloud_filtered += *pointcloud_surf;
      pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
      laserCloudFilteredMsg.header.stamp    = pointcloud_time; // pointcloud header stamp
      laserCloudFilteredMsg.header.frame_id = "base_link";
      pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

      // 发布这两种特征点云
      sensor_msgs::PointCloud2 edgePointsMsg;
      pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
      edgePointsMsg.header.stamp    = pointcloud_time;
      edgePointsMsg.header.frame_id = "base_link";
      pubEdgePoints.publish(edgePointsMsg);

      sensor_msgs::PointCloud2 surfPointsMsg;
      pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
      surfPointsMsg.header.stamp    = pointcloud_time;
      surfPointsMsg.header.frame_id = "base_link";
      pubSurfPoints.publish(surfPointsMsg);
    } // endif: !empty
    // sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  } // endwhile: 1
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  // 设置参数
  int scan_line         = 64; // 雷达线束
  double vertical_angle = 2.0; // 垂直方向两线束的角度，用来计算每个点属于哪个激光线束发射出来的。
  double scan_period    = 0.1; // 扫描周期，0.1s转一圈
  double max_dis        = 60.0; // 点云的有效距离，60M以外噪声较大
  double min_dis        = 2.0;

  nh.getParam("/scan_period", scan_period); // param from launch file
  nh.getParam("/vertical_angle", vertical_angle);
  nh.getParam("/max_dis", max_dis);
  nh.getParam("/min_dis", min_dis);
  nh.getParam("/scan_line", scan_line);

  lidar_param.setScanPeriod(scan_period);
  lidar_param.setVerticalAngle(vertical_angle);
  lidar_param.setLines(scan_line);
  lidar_param.setMaxDistance(max_dis);
  lidar_param.setMinDistance(min_dis);

  laserProcessing.init(lidar_param);
  // callback里面不写逻辑,只装buffer数据
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);

  pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

  pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

  pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);

  // 在process线程里面处理点云数据
  std::thread laser_processing_process{laser_processing}; // 在process线程里面处理点云数据

  ros::spin( );

  return 0;
}
