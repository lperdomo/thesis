//
// Created by phi on 11/04/18.
//

#ifndef PROTOTYPE1_PIPELINE_H
#define PROTOTYPE1_PIPELINE_H

#include <iostream>

#include "io.h"
#include "feature.h"
#include "omp.h"
#include "ahc.h"

namespace prototype1 {

template<class T>
class Pipeline {
public:
  void Run();
  void SetInputParameters(std::string config_file);
  void SetPlaneSegmentationMethod();
  typename IO<T>::Ptr io_;
  typename segmentation::Segmentation<T>::Ptr plane_segmentation_;
};

} // namespace prototype1

#endif //PROTOTYPE1_PIPELINE_H
