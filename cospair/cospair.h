//
// Created by phi on 21/05/18.
//

#ifndef PROTOTYPE2_COSPAIR_H
#define PROTOTYPE2_COSPAIR_H

#include <pcl/features/pfh.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/eigen.h>

#include "pch.h"

namespace prototype2 {
/*
//CoSPAIR : Colored Histograms of Spatial Concentric Surflet-Pairs for 3D object recognition (2016)
//authors Logoglu K. B., Kalkan S., Temizel A.
class CospairDescriptor {
public:
  std::vector<float> descriptor;
};

template<class T>
class CospairEstimation {
public:
  typedef boost::shared_ptr<CospairEstimation<T> > Ptr;
  void Compute(pcl::PointCloud<CospairDescriptor> &descriptors);
  template<class T2>
  void ComputeDescriptor(pcl::PointCloud<CospairDescriptor> &descriptors);
  template<class T2>
  void ComputeColorBins(T p1, T p2, float *color_features);
  void Dimensionate();
  void input_cloud(typename pcl::PointCloud<T>::ConstPtr input_cloud);
  void input_normals(pcl::PointCloud<pcl::Normal>::ConstPtr input_normals);
  void search_surface(typename pcl::PointCloud<T>::ConstPtr search_surface);
  void search_surface_normals(pcl::PointCloud<pcl::Normal>::ConstPtr search_surface_normals);
  void radius(double radius);
  void levels(int levels);
  void depth_bins(int depth_bins);
  void color_bins(int color_bins);
  void color_scheme(int color_scheme);
  void color_scheme(ColorScheme color_scheme);
  void color_l1(bool color_l1);
  int dimension();
  int depth_feature_level_dimension();
  int color_feature_level_dimension();
private:
  typename pcl::PointCloud<T>::ConstPtr input_cloud_;
  pcl::PointCloud<pcl::Normal>::ConstPtr input_normals_;
  typename pcl::PointCloud<T>::ConstPtr search_surface_;
  pcl::PointCloud<pcl::Normal>::ConstPtr search_surface_normals_;
  double radius_;
  int levels_;
  int depth_bins_;
  int color_bins_;
  ColorScheme color_scheme_;
  bool color_l1_;
  int depth_feature_level_dimension_;
  int color_feature_level_dimension_;
};
*/
} // namespace prototype2

#endif //PROTOTYPE2_COSPAIR_H
