//
// Created by phi on 05/07/18.
//

#include "descriptor.h"

namespace prototype2 {

void Descriptor::normal(Normal::Ptr normal) {
  normal_ = normal;
}

Normal::Ptr Descriptor::normal() {
  return normal_;
}

cv::Mat Descriptor::result() {
  return result_;
}

void Descriptor::Extract(pcl::PointCloud<PointType>::ConstPtr cloud) {
  if (normal_) {
    if (cloud->header.seq == 0) normal_->resolution(PointCloudHelper::Resolution(cloud));
    normal_->Estimate(cloud);
    pcl::PointIndices::Ptr indices           = PointCloudHelper::RemoveNaN(normal_->cloud());
    pcl::PointCloud<PointType>::Ptr filtered = PointCloudHelper::Filter<PointType>(cloud, indices, true);
    this->ExtractImpl(filtered);
  } else {
    this->ExtractImpl(cloud);
  }
}

} // namespace prototype2
