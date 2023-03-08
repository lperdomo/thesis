//
// Created by phi on 28/03/18.
//

#include "ahc.h"

namespace prototype1 {
namespace segmentation {

template<class T>
AgglomerativeHierarchicalClustering<T>
::AgglomerativeHierarchicalClustering(int min_support, int window_width
    , int window_height, bool refinement) {
  segmentation_.minSupport   = min_support;
  segmentation_.windowWidth  = window_width;
  segmentation_.windowHeight = window_height;
  segmentation_.doRefine     = refinement;
}

template<class T>
AgglomerativeHierarchicalClustering<T>
::AgglomerativeHierarchicalClustering(typename IO<T>::AhcSegmentation param) {
  segmentation_.minSupport   = param.min_support;
  segmentation_.windowWidth  = param.window_width;
  segmentation_.windowHeight = param.window_height;
  segmentation_.doRefine     = param.refinement;
}

template<class T>
void AgglomerativeHierarchicalClustering<T>
::Compute(typename pcl::PointCloud<T>::Ptr pointcloud) {
  double segmentation_start = pcl::getTime();
  segmented_image_.create(pointcloud->height, pointcloud->width, CV_8UC3);
  PointCloudInterface interface(*pointcloud);
  segmentation_.run(&interface, &membership_indices_, &segmented_image_, 0, false);
  double segmentation_end = pcl::getTime();
  std::cout << "AgglomerativeHierarchicalClustering took "
            << double(segmentation_end - segmentation_start) << "s "
            << std::endl
            << "Detected planes: " << membership_indices_.size()
            << std::endl;

}

template<class T>
cv::Mat AgglomerativeHierarchicalClustering<T>::Result(typename pcl::PointCloud<T>::Ptr pointCloud) {
  cv::Mat image;
  cv::cvtColor(segmented_image_, image, CV_RGB2BGR);
  return image;
}

template<class T>
AgglomerativeHierarchicalClustering<T>::PointCloudInterface
::PointCloudInterface(const pcl::PointCloud<T>& pointcloud)
    : cloud(pointcloud), unitScaleFactor(1) {

}

template<class T>
AgglomerativeHierarchicalClustering<T>::PointCloudInterface
::PointCloudInterface(const PointCloudInterface& pointcloud_interface)
    : cloud(pointcloud_interface.cloud), unitScaleFactor(pointcloud_interface.unitScaleFactor) {

}

template<class T>
int AgglomerativeHierarchicalClustering<T>::PointCloudInterface::width() const {
  return cloud.width;
}

template<class T>
int AgglomerativeHierarchicalClustering<T>::PointCloudInterface::height() const {
  return cloud.height;
}

template<class T>
bool AgglomerativeHierarchicalClustering<T>::PointCloudInterface
::get(const int row, const int col, double& x, double& y, double& z) const {
  const T& pt = cloud.at(col,row);
  x = pt.x*unitScaleFactor;
  y = pt.y*unitScaleFactor;
  z = pt.z*unitScaleFactor;
  return !pcl_isnan(z);
}

template class AgglomerativeHierarchicalClustering<pcl::PointXYZ>;
template class AgglomerativeHierarchicalClustering<pcl::PointXYZRGB>;

} // namespace segmentation
} // namespace prototype1