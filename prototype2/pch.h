//
// Created by phi on 19/07/18.
//

#ifndef PROTOTYPE2_PCH_H
#define PROTOTYPE2_PCH_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types_conversion.h>

#include <pcl/io/pcd_io.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/search/kdtree.h>

#include "util.h"

namespace prototype2 {

class PointCloudHelper {
public:
  template<class T>
  static typename pcl::PointCloud<T>::Ptr MakePointCloud(T point);
  template<class T>
  static typename pcl::PointCloud<T>::Ptr Filter(typename pcl::PointCloud<T>::ConstPtr cloud, pcl::PointIndices::ConstPtr indices, bool filter_out);
  template<class T>
  static T Centroid(typename pcl::PointCloud<T>::ConstPtr cloud);
  static pcl::PointIndices::Ptr RemoveNaN(pcl::PointCloud<pcl::Normal>::Ptr cloud);
  static pcl::PointIndices::Ptr RemoveNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  static pcl::PointIndices::Ptr RemoveNaN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  static pcl::PointCloud<PointType>::Ptr Sample(pcl::PointCloud<PointType>::ConstPtr cloud, float leaf_size);
  static float MaxDistance(PointType point, pcl::PointCloud<PointType>::ConstPtr cloud);
  static double Resolution(pcl::PointCloud<PointType>::ConstPtr cloud);

};

} // namespace prototype

#endif //PROTOTYPE2_PCH_H
