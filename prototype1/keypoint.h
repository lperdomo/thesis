//
// Created by phi on 05/07/18.
//

#ifndef PROTOTYPE2_KEYPOINT_H
#define PROTOTYPE2_KEYPOINT_H

#include <pcl/search/kdtree.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>

#include "util.h"
#include "pch.h"

namespace prototype2 {

class Harris3d {
public:
  typedef pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI, pcl::Normal> Method;
  static int scalar_radius;
  static constexpr int id = 1;
};

class Iss3d {
public:
  typedef pcl::ISSKeypoint3D<PointType, PointType> Method;
  static float threshold21;
  static float threshold32;
  static int min_neighbors;
  static int scalar_salient_radius;
  static int scalar_non_max_radius;
  static int scalar_normal_radius;
  static int scalar_border_radius;
  static constexpr int id = 2;
};

class Keypoint {
public:
  typedef boost::shared_ptr<Keypoint> Ptr;
  Keypoint();
  void Detect(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals = 0);
  virtual void SetParameters(double resolution) = 0;
  pcl::PointCloud<PointType>::Ptr cloud();
protected:
  pcl::PointCloud<PointType>::Ptr cloud_;
  virtual void DetectImpl(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals) = 0;
};

template<class T>
class KeypointEstimation : public Keypoint {
public:
  typedef boost::shared_ptr<KeypointEstimation<T> > Ptr;
  void SetParameters(double resolution) override;
private:
  typename T::Method method_;
  void DetectImpl(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals) override;
};

} // namespace prototype2

#endif //PROTOTYPE2_KEYPOINT_H
