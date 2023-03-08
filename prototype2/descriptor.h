//
// Created by phi on 05/07/18.
//

#ifndef PROTOTYPE2_DESCRIPTOR_H
#define PROTOTYPE2_DESCRIPTOR_H

#include "util.h"
#include "pch.h"
#include "normal.h"

#include <pcl/visualization/pcl_visualizer.h>

namespace prototype2 {

class Descriptor {
public:
  typedef boost::shared_ptr<Descriptor> Ptr;
  virtual void ExtractImpl(pcl::PointCloud<PointType>::ConstPtr cloud) = 0;
  void Extract(pcl::PointCloud<PointType>::ConstPtr cloud);
  void normal(Normal::Ptr normal);
  Normal::Ptr normal();
  cv::Mat result();
protected:
  Normal::Ptr normal_;
  cv::Mat result_;
};

} // namespace prototype2

#endif //PROTOTYPE2_DESCRIPTOR_H
