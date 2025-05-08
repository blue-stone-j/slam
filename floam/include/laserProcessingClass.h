/*
Description:
Author     : Ji Qingshi
date       :
*/

#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
/* 如果是线结构光的采集方式得到的点云,则沿z向的分布较广,但沿x、y方向的分布则处于有限的范围内。
   此时,可采用直通滤波,确定x或者y方向的范围,快速裁剪离群点。*/
#include <pcl/filters/passthrough.h> // 直通滤波
#include <pcl/filters/extract_indices.h> // 滤波器,从点云中提取指定索引的点作为一个新的点云
#include <pcl/filters/crop_box.h> // 一个过滤器,允许用户过滤给定框内的所有数据,可以实现将给定的框范围内和范围外的点进行提取保存。

#include "lidar.h"

// points covariance class
class Double2d
{ // point id and corresponding value, for example, cloudCurvature
 public:
  int id;
  double value;
  Double2d(int id_in, double value_in);
};
// points info class(unused)
class PointsInfo
{
 public:
  int layer; // ???
  double time; //???
  PointsInfo(int layer_in, double time_in);
};

class LaserProcessingClass
{
 public:
  LaserProcessingClass( );
  void init(lidar::Lidar lidar_param_in);
  void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf);
  void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                   std::vector<Double2d> &cloudCurvature,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf);

 private:
  lidar::Lidar lidar_param;
};

#endif // _LASER_PROCESSING_CLASS_H_
