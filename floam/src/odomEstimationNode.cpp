// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ros lib
#include <ros/ros.h>
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
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  mutex_lock.lock( );
  pointCloudSurfBuf.push(laserCloudMsg);
  mutex_lock.unlock( );
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  mutex_lock.lock( );
  pointCloudEdgeBuf.push(laserCloudMsg);
  mutex_lock.unlock( );
}

bool is_odom_inited = false;
double total_time   = 0;
int total_frame     = 0;
void odom_estimation( )
{
  while (1)
  {
    if (!pointCloudEdgeBuf.empty( ) && !pointCloudSurfBuf.empty( ))
    {
      // read data from buffer
      // 注意点云的时间戳同步， 0.5*scan_period时间内的认为是一个位置的点云，不合条件的pop掉
      mutex_lock.lock( );
      if (!pointCloudSurfBuf.empty( )
          && (pointCloudSurfBuf.front( )->header.stamp.toSec( ) < pointCloudEdgeBuf.front( )->header.stamp.toSec( ) - 0.5 * lidar_param.scan_period))
      {
        pointCloudSurfBuf.pop( );
        ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock.unlock( );
        continue;
      }

      if (!pointCloudEdgeBuf.empty( )
          && (pointCloudEdgeBuf.front( )->header.stamp.toSec( ) < pointCloudSurfBuf.front( )->header.stamp.toSec( ) - 0.5 * lidar_param.scan_period))
      {
        pointCloudEdgeBuf.pop( );
        ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock.unlock( );
        continue;
      }
      // if time aligned

      // 把同步好的对应点云帧取出来
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>( ));
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>( ));
      pcl::fromROSMsg(*pointCloudEdgeBuf.front( ), *pointcloud_edge_in);
      pcl::fromROSMsg(*pointCloudSurfBuf.front( ), *pointcloud_surf_in);
      ros::Time pointcloud_time = (pointCloudSurfBuf.front( ))->header.stamp;
      pointCloudEdgeBuf.pop( );
      pointCloudSurfBuf.pop( );
      mutex_lock.unlock( );

      // 第一帧直接加入map,否则更新map
      if (is_odom_inited == false)
      {
        // 分别将当前特征点云加入到相应的特征点局部地图中
        odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
        is_odom_inited = true;
        ROS_INFO("odom inited");
      }
      else
      {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now( );
        // 构建优化问题并求解、更新odom
        odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
        end                                          = std::chrono::system_clock::now( );
        std::chrono::duration<float> elapsed_seconds = end - start;
        total_frame++;
        float time_temp = elapsed_seconds.count( ) * 1000;
        total_time += time_temp;
        ROS_INFO("average odom estimation time %f ms \n \n", total_time / total_frame);
      }

      Eigen::Quaterniond q_current(odomEstimation.odom.rotation( ));
      // q_current.normalize();
      Eigen::Vector3d t_current = odomEstimation.odom.translation( );

      // 发布map->base_link的tf
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(t_current.x( ), t_current.y( ), t_current.z( )));
      tf::Quaternion q(q_current.x( ), q_current.y( ), q_current.z( ), q_current.w( ));
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now( ), "map", "base_link"));

      // publish odometry
      nav_msgs::Odometry laserOdometry;
      laserOdometry.header.frame_id         = "map";
      laserOdometry.child_frame_id          = "base_link";
      laserOdometry.header.stamp            = pointcloud_time;
      laserOdometry.pose.pose.orientation.x = q_current.x( );
      laserOdometry.pose.pose.orientation.y = q_current.y( );
      laserOdometry.pose.pose.orientation.z = q_current.z( );
      laserOdometry.pose.pose.orientation.w = q_current.w( );
      laserOdometry.pose.pose.position.x    = t_current.x( );
      laserOdometry.pose.pose.position.y    = t_current.y( );
      laserOdometry.pose.pose.position.z    = t_current.z( );
      pubLaserOdometry.publish(laserOdometry);
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

  // 参数设置
  int scan_line         = 64;
  double vertical_angle = 2.0;
  double scan_period    = 0.1;
  double max_dis        = 60.0;
  double min_dis        = 2.0;
  double map_resolution = 0.4;
  nh.getParam("/scan_period", scan_period);
  nh.getParam("/vertical_angle", vertical_angle);
  nh.getParam("/max_dis", max_dis);
  nh.getParam("/min_dis", min_dis);
  nh.getParam("/scan_line", scan_line);
  nh.getParam("/map_resolution", map_resolution);

  lidar_param.setScanPeriod(scan_period);
  lidar_param.setVerticalAngle(vertical_angle);
  lidar_param.setLines(scan_line);
  lidar_param.setMaxDistance(max_dis);
  lidar_param.setMinDistance(min_dis);

  odomEstimation.init(lidar_param, map_resolution);
  // 接收特征点云，装进buffer中
  ros::Subscriber subEdgeLaserCloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
  ros::Subscriber subSurfLaserCloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

  pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
  // 业务逻辑在odom_estimation线程中处理，发布一个estimate的odometry
  std::thread odom_estimation_process{odom_estimation};

  ros::spin( );

  return 0;
}
