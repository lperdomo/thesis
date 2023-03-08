//
// Created by phi on 21/03/18.
//

#ifndef PROTOTYPE1_SEGMENTATION_H
#define PROTOTYPE1_SEGMENTATION_H

#include <opencv2/opencv.hpp>

#include <pcl/common/time.h>

#include "feature.h"

namespace prototype1 {
namespace segmentation {

template<class T>
class Segmentation {
public:
  typedef boost::shared_ptr<Segmentation<T> > Ptr;

  virtual void Compute(typename pcl::PointCloud<T>::Ptr pointCloud) = 0;
  virtual cv::Mat Result(typename pcl::PointCloud<T>::Ptr pointCloud) = 0;

};

} // namespace segmentation
} // namespace prototype1

#endif //PROTOTYPE1_SEGMENTATION_H
