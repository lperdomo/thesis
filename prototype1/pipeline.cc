#include "pipeline.h"

namespace prototype1 {

template<class T>
void Pipeline<T>::Run() {
  this->SetPlaneSegmentationMethod();
  int i = 0;
  while (i < io_->config_.dataset.size) {
    typename pcl::PointCloud<T>::Ptr pointcloud;
    pointcloud.reset(new pcl::PointCloud<T>);

    io_->LoadPointCloud(pointcloud, i);

    plane_segmentation_->Compute(pointcloud);

    io_->Show("Segmentation", plane_segmentation_->Result(pointcloud));

    i++;

  }

}

template<class T>
void Pipeline<T>::SetInputParameters(std::string config_file) {
  io_.reset(new prototype1::IO<T>(config_file));
}

template<class T>
void Pipeline<T>::SetPlaneSegmentationMethod() {
  if (io_->config_.ahc_segmentation.active) {
    plane_segmentation_.reset(new segmentation::AgglomerativeHierarchicalClustering<T>
                            (io_->config_.ahc_segmentation));
  } else if (io_->config_.omp_segmentation.active) {
    typename Feature<T>::Ptr feature;
    feature.reset(new Feature<T>(io_->config_.normal_estimation));
    plane_segmentation_.reset(new segmentation::OrganizedMultiPlane<T>
                               (io_->config_.omp_segmentation, feature));
  }
}

} // namespace prototype1

int main(int argc, char **argv) {
  prototype1::Pipeline<pcl::PointXYZRGB> pipeline;
  pipeline.SetInputParameters(std::string(argv[1]));
  pipeline.Run();

  return 0;
}