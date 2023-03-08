//
// Created by phi on 22/03/18.
//

#ifndef PROTOTYPE1_FEATURE_H
#define PROTOTYPE1_FEATURE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/integral_image_normal.h>
#include "io.h"

namespace prototype1 {

template<class T>
class Feature {
public:
  typedef boost::shared_ptr<Feature<T> > Ptr;
  Feature(float max_depth_change_factor, float normal_smoothing_size);
  Feature(typename IO<T>::NormalEstimation param);
  void NormalEstimation(typename pcl::PointCloud<T>::Ptr pointcloud);
  pcl::IntegralImageNormalEstimation<T, pcl::Normal> normal_estimation;
  pcl::PointCloud<pcl::Normal>::Ptr normalcloud_;
};

} // namespace prototype1

#endif //PROTOTYPE1_FEATURE_H
