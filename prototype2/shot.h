//
// Created by phi on 21/05/18.
//

#ifndef PROTOTYPE2_SHOT_H
#define PROTOTYPE2_SHOT_H

#include <iostream>
#include <fstream>

#include <pcl/features/shot.h>

#include "pch.h"
#include "descriptor.h"

namespace prototype2 {

struct Shot {
  typedef pcl::SHOT352 Type;
  typedef pcl::SHOTEstimation<PointType, pcl::Normal, Shot::Type> Method;
};
struct CShot {
  typedef pcl::SHOT1344 Type;
  typedef pcl::SHOTColorEstimation<PointType, pcl::Normal, CShot::Type> Method;
};

template<typename T>
class ShotEstimation : public Descriptor {
public:
  typedef boost::shared_ptr<ShotEstimation<T> > Ptr;
  ShotEstimation(pcl::search::KdTree<PointType>::Ptr kdtree);
  void ExtractImpl(pcl::PointCloud<PointType>::ConstPtr cloud) override;
private:
  typename T::Method method_;
};

template class ShotEstimation<Shot>;
template class ShotEstimation<CShot>;

} // namespace prototype2

#endif //PROTOTYPE2_SHOT_H
