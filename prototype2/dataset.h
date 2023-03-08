//
// Created by phi on 18/07/18.
//

#ifndef PROTOTYPE2_DATASET_H
#define PROTOTYPE2_DATASET_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <fstream>

#include "util.h"
#include "pch.h"

namespace prototype2 {

class Dataset {
public:
  typedef boost::shared_ptr<Dataset> Ptr;
  Dataset();
  pcl::PointCloud<PointType>::Ptr Frame(int n);
  void Save(boost::shared_ptr<cv::Mat> descriptors);
  void Load(boost::shared_ptr<cv::Mat> descriptors,
            boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > pose);
  void path(std::string path);
  void frames(std::string frames);
  void poses(std::string poses);
  void timestamps(std::string timestamps);
  void output(std::string output);
  void format(std::string format);
  void amount(int amount);
  void distance(int distance);
  void mask(int mask);
  void downsample(int downsample);
  std::string output();
  int amount();
  int distance();
private:
  std::string path_;
  std::string frames_;
  std::string poses_;
  std::string timestamps_;
  std::string output_;
  std::string format_;
  float resolution_;
  int amount_;
  int mask_;
  int downsample_;
  int distance_;
};

} // namespace prototype2

#endif //PROTOTYPE2_DATASET_H
