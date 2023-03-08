//
// Created by phi on 22/03/18.
//

#include <pcl/common/time.h>

#include "feature.h"

namespace prototype1 {

template<class T>
Feature<T>::Feature(typename IO<T>::NormalEstimation param) {
  normal_estimation.setMaxDepthChangeFactor(param.max_depth_change_factor);
  normal_estimation.setNormalSmoothingSize(param.normal_smoothing_size);
}

template<class T>
Feature<T>::Feature(float max_depth_change_factor, float normal_smoothing_size) {
  normal_estimation.setMaxDepthChangeFactor(max_depth_change_factor);
  normal_estimation.setNormalSmoothingSize(normal_smoothing_size);
}

template<class T>
void Feature<T>::NormalEstimation(typename pcl::PointCloud<T>::Ptr pointcloud) {
  double estimation_start = pcl::getTime();
  normalcloud_.reset(new pcl::PointCloud<pcl::Normal>);
  normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
  normal_estimation.setDepthDependentSmoothing(true);
  normal_estimation.setInputCloud(pointcloud);
  normal_estimation.compute(*normalcloud_);
  double estimation_end = pcl::getTime();
  std::cout << "NormalEstimation took "
            << double(estimation_end - estimation_start)
            << std::endl;
}

template class Feature<pcl::PointXYZ>;
template class Feature<pcl::PointXYZRGB>;

} // namespace prototype1
