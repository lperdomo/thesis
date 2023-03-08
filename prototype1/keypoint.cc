//
// Created by phi on 05/07/18.
//

#include "keypoint.h"

namespace prototype2 {

int Harris3d::scalar_radius = 0;

float Iss3d::threshold21         = 0.f;
float Iss3d::threshold32         = 0.f;
int Iss3d::min_neighbors         = 0;
int Iss3d::scalar_salient_radius = 0;
int Iss3d::scalar_non_max_radius = 0;
int Iss3d::scalar_normal_radius  = 0;
int Iss3d::scalar_border_radius  = 0;


Keypoint::Keypoint() : cloud_(boost::make_shared<pcl::PointCloud<PointType> >()) {
}

pcl::PointCloud<PointType>::Ptr Keypoint::cloud() {
  return cloud_;
}

void Keypoint::Detect(pcl::PointCloud<PointType>::ConstPtr cloud,
                      pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
  this->DetectImpl(cloud, normals);
}

template<>
void KeypointEstimation<Harris3d>::DetectImpl(pcl::PointCloud<PointType>::ConstPtr cloud,
                                              pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
  pcl::PointCloud<pcl::PointXYZI> output;
  method_.setNormals(normals);
  method_.setInputCloud(cloud);
  method_.compute(output);
  pcl::copyPointCloud(output, *cloud_);
}

template<>
void KeypointEstimation<Iss3d>::DetectImpl(pcl::PointCloud<PointType>::ConstPtr cloud,
                                           pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
  method_.setInputCloud(cloud);
  method_.compute(*cloud_);
}

template<>
void KeypointEstimation<Harris3d>::SetParameters(double resolution) {
  method_.setRadius(static_cast<float>(Harris3d::scalar_radius * resolution));
}

template<>
void KeypointEstimation<Iss3d>::SetParameters(double resolution) {
  method_.setThreshold21(Iss3d::threshold21);
  method_.setThreshold32(Iss3d::threshold32);
  method_.setMinNeighbors(Iss3d::min_neighbors);
  method_.setSalientRadius(Iss3d::scalar_salient_radius * resolution);
  method_.setNonMaxRadius(Iss3d::scalar_non_max_radius  * resolution);
  method_.setNormalRadius(Iss3d::scalar_normal_radius   * resolution);
  method_.setBorderRadius(Iss3d::scalar_border_radius   * resolution);
}

} // namespace prototype2
