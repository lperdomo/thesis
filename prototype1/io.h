//
// Created by phi on 20/03/18.
//

#ifndef PROTOTYPE1_IO_H
#define PROTOTYPE1_IO_H

#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

namespace prototype1 {

template<class T>
class IO {
public:
  IO(std::string configFile);
  struct Dataset {
    int size;
    float width;
    float height;
    std::string reference;
    std::string path;
  };
  struct NormalEstimation {
    float max_depth_change_factor;
    float normal_smoothing_size;
  };
  struct OmpSegmentation {
    bool active;
    unsigned int min_inliers;
    double angular_threshold;
    double distance_threshold;
  };
  struct AhcSegmentation {
    bool active;
    bool refinement;
    int min_support;
    int window_width;
    int window_height;
  };
  struct Configuration {
    Dataset dataset;
    NormalEstimation normal_estimation;
    OmpSegmentation omp_segmentation;
    AhcSegmentation ahc_segmentation;
  };
  typedef boost::shared_ptr<IO<T> > Ptr;
  void LoadConfigFile(std::string file);
  void AssertFileExists(std::string file);
  void LoadPointCloud(typename pcl::PointCloud<T>::Ptr pointcloud
      , int i);
  void LoadPointCloud(typename pcl::PointCloud<T>::Ptr pointcloud
      , std::string filename);
  void LoadDatasetReference(int index);
  void Show(std::string title, cv::Mat image);
  Configuration config_;
private:
  std::string reference_rgb_;
  std::string reference_depth_;
};

} // namespace prototype1

#endif //IO_H
