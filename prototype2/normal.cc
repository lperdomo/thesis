//
// Created by phi on 04/07/18.
//

#include "normal.h"

namespace prototype2 {

int NormalEstimation::scalar_radius = 0;

Normal::Normal() : cloud_(boost::make_shared<pcl::PointCloud<pcl::Normal> >()) {
}

NormalEstimation::NormalEstimation(pcl::search::KdTree<PointType>::Ptr kdtree) : Normal() {
  method_.setSearchMethod(kdtree);
}


pcl::PointCloud<pcl::Normal>::Ptr Normal::cloud() {
  return cloud_;
}

void NormalEstimation::Estimate(pcl::PointCloud<PointType>::ConstPtr cloud) {
  //double time_start = pcl::getTime();
  method_.setInputCloud(cloud);
  method_.compute(*cloud_);
  //std::cout << "Normal estimation took " << double(pcl::getTime() - time_start) << std::endl;
}

void NormalEstimation::resolution(double resolution) {
  method_.setRadiusSearch(scalar_radius * resolution);
}

/*
template class Normal<LeastSquares>;
template class Normal<IntegralImage>;

pcl::PointCloud<pcl::Normal>::Ptr NormalBase::cloud() {
  return cloud_;
}

template<class T>
Normal<T>::Normal(T method) : method_(method) {

}

template<class T>
void Normal<T>::Estimate(pcl::PointCloud<PointType>::ConstPtr cloud) {
  double time_start = pcl::getTime();
  cloud_.reset(new pcl::PointCloud<pcl::Normal>);
  method_.setInputCloud(cloud);
  method_.compute(*cloud_);
  double time_end = pcl::getTime();
  std::cout << "NormalEstimation took "
            << double(time_end - time_start)
            << std::endl;
}
*/
} // namespace prototype2
