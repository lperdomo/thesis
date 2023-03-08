//
// Created by phi on 28/03/18.
//

#ifndef PROTOTYPE1_OMP_H
#define PROTOTYPE1_OMP_H

#include <opencv2/opencv.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <random>

#include "segmentation.h"
#include "feature.h"
#include "io.h"

namespace prototype1 {
namespace segmentation {

//ICRAW 2013
//Efficient Organized Point Cloud Segmentation with Connected Components (2013)
//authors Alexander J. B. Trevor, Suat Gedikli, Radu B. Rusu, Henrik I. Christensen
template<class T>
class OrganizedMultiPlane : public Segmentation<T> {
public:
  typedef boost::shared_ptr<OrganizedMultiPlane<T> > Ptr;
  OrganizedMultiPlane(typename IO<T>::OmpSegmentation param, typename prototype1::Feature<T>::Ptr feature);
  OrganizedMultiPlane(unsigned int min_inliers, double angular_threshold, double distance_threshold, typename prototype1::Feature<T>::Ptr feature);
  void Compute(typename pcl::PointCloud<T>::Ptr pointcloud) override;
  void Compute(typename pcl::PointCloud<T>::Ptr pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normalCloud);
  cv::Mat Result(typename pcl::PointCloud<T>::Ptr pointcloud) override;
  void Clear();
  typename pcl::OrganizedMultiPlaneSegmentation<T, pcl::Normal, pcl::Label> segmentation_;
  typename prototype1::Feature<T>::Ptr feature_;
  pcl::PointCloud<pcl::Label>::Ptr labels_;
  std::vector<pcl::PointIndices> label_indices_;
  std::vector<pcl::PointIndices> inlier_indices_;
  std::vector<pcl::PointIndices> boundary_indices_;
  std::vector<pcl::ModelCoefficients> model_coefficients_;
  std::vector<pcl::PlanarRegion<T>, Eigen::aligned_allocator<pcl::PlanarRegion<T> > > regions_;
};

} // namespace segmentation
} // namespace prototype1

#endif //PROTOTYPE1_OMP_H
