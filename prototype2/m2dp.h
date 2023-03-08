//
// Created by phi on 21/05/18.
//

#ifndef PROTOTYPE2_M2DP_H
#define PROTOTYPE2_M2DP_H

#include <iostream>
#include <fstream>

#include <pcl/common/pca.h>
#include <opencv2/core/eigen.hpp>

#include "pch.h"
#include "descriptor.h"

namespace prototype2 {

/** M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop Closure Detection (2016)
  * Authors Li He, Xiaolong Wang and Hong Zhang
  * https://github.com/LiHeUA/M2DP
  */
struct M2dp {
  /** number of bins in theta, the 't' in paper */
  static int theta_bins;
  /** number of bins in rho, the 'l' in paper */
  static int rho_bins;
  /** number of azimuth angles, the 'p' in paper */
  static int azimuth_angles;
  /** number of elevation angles, the 'q' in paper */
  static int elevation_angles;
};
struct CM2dp : M2dp {
  static int color_bins;
  static int color_scheme;
};


template<typename T>
class M2dpEstimation : public Descriptor {
public:
  typedef boost::shared_ptr<M2dpEstimation<T> > Ptr;
  struct PolarY {
    EIGEN_EMPTY_STRUCT_CTOR(PolarY)
    typedef float result_type;
    float operator() (const float& a, const float& b) const { return (std::sqrt(std::pow(a,2) + std::pow(b,2))); }
  };
  struct PolarX {
    EIGEN_EMPTY_STRUCT_CTOR(PolarX)
    typedef float result_type;
    float operator() (const float& a, const float& b) const { return (std::atan2(a,b)); }
  };
  M2dpEstimation();
  void ExtractImpl(pcl::PointCloud<PointType>::ConstPtr cloud) override;
  int color_stride_channel_1_;
  int color_stride_channel_2_;
  int color_stride_channel_3_;
  float color_bins_;
  int rgb_bins_lut_[256];
};

template class M2dpEstimation<M2dp>;
template class M2dpEstimation<CM2dp>;

} // namespace prototype2

#endif //PROTOTYPE2_M2DP_H
