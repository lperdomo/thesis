//
// Created by phi on 28/03/18.
//

#ifndef PROTOTYPE1_AHC_H
#define PROTOTYPE1_AHC_H

#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "peac/AHCPlaneFitter.hpp"

#include "segmentation.h"
#include "io.h"

namespace prototype1 {
namespace segmentation {

//ICRA 2014
//Fast Plane Extraction in Organized Point Clouds Using Agglomerative Hierarchical Clustering
//authors Feng C., Taguchi Y., Kamat V.
template<class T>
class AgglomerativeHierarchicalClustering : public Segmentation<T> {
public:
  class PointCloudInterface {
  public:
    PointCloudInterface(const pcl::PointCloud<T>& c);
    PointCloudInterface(const PointCloudInterface& other);
    bool get(const int row, const int col, double& x, double& y, double& z) const;
    int width() const;
    int height() const;
    const pcl::PointCloud<T>& cloud;
    const double unitScaleFactor; //mm
  };
  typedef boost::shared_ptr<AgglomerativeHierarchicalClustering<T> > Ptr;
  AgglomerativeHierarchicalClustering(typename IO<T>::AhcSegmentation param);
  AgglomerativeHierarchicalClustering(int min_support, int window_width
      , int window_height, bool refinement);
  void Compute(typename pcl::PointCloud<T>::Ptr pointCloud) override;
  cv::Mat Result(typename pcl::PointCloud<T>::Ptr pointCloud) override;
  ahc::PlaneFitter<PointCloudInterface> segmentation_;
  std::vector<std::vector<int> > membership_indices_;
private:
  cv::Mat segmented_image_;
};

} // namespace segmentation
} // namespace prototype1

#endif //PROTOTYPE1_AHC_H
