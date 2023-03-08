//
// Created by phi on 04/07/18.
//

#ifndef PROTOTYPE2_NORMAL_H
#define PROTOTYPE2_NORMAL_H

#include <pcl/common/time.h>

#include <pcl/search/kdtree.h>

#include <pcl/filters/filter.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include "util.h"

namespace prototype2 {

class Normal {
public:
  typedef boost::shared_ptr<Normal> Ptr;
  Normal();
  virtual void Estimate(pcl::PointCloud<PointType>::ConstPtr cloud) = 0;
  virtual void resolution(double resolution) = 0;
  pcl::PointCloud<pcl::Normal>::Ptr cloud();
protected:
  pcl::PointCloud<pcl::Normal>::Ptr cloud_;
};

class NormalEstimation : public Normal {
public:
  typedef boost::shared_ptr<NormalEstimation> Ptr;
  static int scalar_radius;
  NormalEstimation(pcl::search::KdTree<PointType>::Ptr kdtree);
  void Estimate(pcl::PointCloud<PointType>::ConstPtr cloud) override;
  void resolution(double resolution) override;
private:
  pcl::NormalEstimation<PointType, pcl::Normal> method_;
};

} // namespace prototype2

#endif //PROTOTYPE2_NORMAL_H
