// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"

void LaserProcessingClass::init(lidar::Lidar lidar_param_in)
{
  lidar_param = lidar_param_in;
}

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge,
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf)
{
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc_in, indices); // 输入点云及对应保留的索引, index of valid points

  // 每一圈线上的点云取出来处理laserCloudScans[scanID]
  int N_SCANS = lidar_param.num_lines;
  // a series of pointclouds; every pointcloud is a scan/row of original cloud
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
  // initilize laserCloudScans
  for (int i = 0; i < N_SCANS; i++)
  {
    laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>( )));
  }

  // 每一个点，根据距离和垂直角度，计算是属于哪一个scan上面，其中velodyne雷达的点云会自己返回scan id不用算。
  // extract points into corresponding scan
  for (int i = 0; i < (int)pc_in->points.size( ); i++)
  {
    int scanID = 0;
    // like as method of extracting ground in lego-loam, extract feature from every sector
    // so distance from xy, angle from z and distance
    double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
    // not to extract features from points beyond this range
    if (distance < lidar_param.min_distance || distance > lidar_param.max_distance)
    {
      continue;
    }
    // 计算点在垂直方向的角度,用来计算属于哪个线束
    double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;
    // valid angle
    if (N_SCANS == 16) // 16线两线束间间隔30度,从-15到+15共16根线
    {
      scanID = int((angle + 15) / 2 + 0.5); // 这里为了避免出现负数,统一加15度
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        continue;
      }
    }
    else if (N_SCANS == 32)
    {
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        continue;
      }
    }
    else if (N_SCANS == 64)
    {
      if (angle >= -8.83)
      {
        scanID = int((2 - angle) * 3.0 + 0.5);
      }
      else
      {
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
      }
      // invalid angle and scanID
      if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
      {
        continue;
      }
    }
    else
    {
      printf("wrong scan number\n");
    }
    laserCloudScans[scanID]->push_back(pc_in->points[i]); // 将每个点装入不同的容器

  } // endfor: have extracted points into corresponding scan

  // now we have removed out-range points
  // calculate curve of ecery point in every line, same as in loam
  for (int i = 0; i < N_SCANS; i++)
  {
    if (laserCloudScans[i]->points.size( ) < 131) // points are too few
    {
      continue;
    }

    // 每一条线首尾5个点舍弃，计算连续10个点的x,y,z距离
    std::vector<Double2d> cloudCurvature; // Double2d 是自定义的结构体，点云信息+曲率
    int total_points = laserCloudScans[i]->points.size( ) - 10; // dont extract margin points
    // 当前点前后各五个点,计算其三个方向上距离和的平方
    for (int j = 5; j < (int)laserCloudScans[i]->points.size( ) - 5; j++)
    {
      double diffX = laserCloudScans[i]->points[j - 5].x
                     + laserCloudScans[i]->points[j - 4].x
                     + laserCloudScans[i]->points[j - 3].x
                     + laserCloudScans[i]->points[j - 2].x
                     + laserCloudScans[i]->points[j - 1].x
                     - 10 * laserCloudScans[i]->points[j].x
                     + laserCloudScans[i]->points[j + 1].x
                     + laserCloudScans[i]->points[j + 2].x
                     + laserCloudScans[i]->points[j + 3].x
                     + laserCloudScans[i]->points[j + 4].x
                     + laserCloudScans[i]->points[j + 5].x;
      double diffY = laserCloudScans[i]->points[j - 5].y
                     + laserCloudScans[i]->points[j - 4].y
                     + laserCloudScans[i]->points[j - 3].y
                     + laserCloudScans[i]->points[j - 2].y
                     + laserCloudScans[i]->points[j - 1].y
                     - 10 * laserCloudScans[i]->points[j].y
                     + laserCloudScans[i]->points[j + 1].y
                     + laserCloudScans[i]->points[j + 2].y
                     + laserCloudScans[i]->points[j + 3].y
                     + laserCloudScans[i]->points[j + 4].y
                     + laserCloudScans[i]->points[j + 5].y;
      double diffZ = laserCloudScans[i]->points[j - 5].z
                     + laserCloudScans[i]->points[j - 4].z
                     + laserCloudScans[i]->points[j - 3].z
                     + laserCloudScans[i]->points[j - 2].z
                     + laserCloudScans[i]->points[j - 1].z
                     - 10 * laserCloudScans[i]->points[j].z
                     + laserCloudScans[i]->points[j + 1].z
                     + laserCloudScans[i]->points[j + 2].z
                     + laserCloudScans[i]->points[j + 3].z
                     + laserCloudScans[i]->points[j + 4].z
                     + laserCloudScans[i]->points[j + 5].z;
      Double2d distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
      cloudCurvature.push_back(distance);
    }

    // 将点云均匀划分成6个子图,保证各方向都能提取到特征
    for (int j = 0; j < 6; j++)
    {
      int sector_length = (int)(total_points / 6);
      int sector_start  = sector_length * j;
      int sector_end    = sector_length * (j + 1) - 1;
      if (j == 5) // for last sector
      {
        sector_end = total_points - 1;
      }
      // save this interval to sub
      std::vector<Double2d> subCloudCurvature(cloudCurvature.begin( ) + sector_start, cloudCurvature.begin( ) + sector_end);

      // 特征提取
      featureExtractionFromSector(laserCloudScans[i], subCloudCurvature, pc_out_edge, pc_out_surf);
    } // endfor: have processed all six sub-images

  } // endfor: have extracted all features
}

void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, std::vector<Double2d> &cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf)
{
  // 将点云按曲率大小进行排序
  std::sort(cloudCurvature.begin( ), cloudCurvature.end( ),
            [](const Double2d &a, const Double2d &b) { return a.value < b.value; });

  int largestPickedNum = 0;
  std::vector<int> picked_points; // store points that have been picked as corner or surf
  int point_info_count = 0;
  // 遍历点云,从曲率大的点开始提,根据曲率判断其是平面点还是边缘点, pick edge feature points
  for (int i = cloudCurvature.size( ) - 1; i >= 0; i--)
  {
    int ind = cloudCurvature[i].id;
    // if this point haven't been picked
    if (std::find(picked_points.begin( ), picked_points.end( ), ind) == picked_points.end( ))
    {
      // max Curvature is too small, means no edge points, break
      if (cloudCurvature[i].value <= 0.1)
      {
        break;
      }

      largestPickedNum++;
      picked_points.push_back(ind);

      // 找到曲率最大的20个点; whether pick enough points
      if (largestPickedNum <= 20)
      {
        pc_out_edge->push_back(pc_in->points[ind]);
        point_info_count++;
      }
      else
      {
        break;
      }
      // pick neighbor points that may locate in same edge feature with this edge point
      // 计算此点前后5个点(的曲率),与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
      for (int k = 1; k <= 5; k++)
      {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
        {
          break;
        }
        picked_points.push_back(ind + k);
      }
      for (int k = -1; k >= -5; k--)
      {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
        {
          break;
        }
        picked_points.push_back(ind + k);
      }
    } // endif: this point haven't been picked
  } // endfor: have picked edge feature points

  // find flat points
  //  point_info_count =0;
  //  int smallestPickedNum = 0;

  // for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
  // {
  //     int ind = cloudCurvature[i].id;

  //     if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
  //         if(cloudCurvature[i].value > 0.1){
  //             //ROS_WARN("extracted feature not qualified, please check lidar");
  //             break;
  //         }
  //         smallestPickedNum++;
  //         picked_points.push_back(ind);

  //         if(smallestPickedNum <= 4){
  //             //find all points
  //             pc_surf_flat->push_back(pc_in->points[ind]);
  //             pc_surf_lessFlat->push_back(pc_in->points[ind]);
  //             point_info_count++;
  //         }
  //         else{
  //             break;
  //         }

  //         for(int k=1;k<=5;k++){
  //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
  //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
  //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
  //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
  //                 break;
  //             }
  //             picked_points.push_back(ind+k);
  //         }
  //         for(int k=-1;k>=-5;k--){
  //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
  //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
  //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
  //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
  //                 break;
  //             }
  //             picked_points.push_back(ind+k);
  //         }

  //     }
  // }

  // 平面点(all the rest point are saved as surf)
  for (int i = 0; i <= (int)cloudCurvature.size( ) - 1; i++)
  {
    int ind = cloudCurvature[i].id;
    if (std::find(picked_points.begin( ), picked_points.end( ), ind) == picked_points.end( ))
    {
      pc_out_surf->push_back(pc_in->points[ind]);
    }
  }
}
LaserProcessingClass::LaserProcessingClass( )
{
}

Double2d::Double2d(int id_in, double value_in)
{
  id    = id_in;
  value = value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in)
{
  layer = layer_in;
  time  = time_in;
};
