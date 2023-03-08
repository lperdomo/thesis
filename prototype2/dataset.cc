//
// Created by phi on 18/07/18.
//

#include "dataset.h"

namespace prototype2 {

Dataset::Dataset() {
}

void Dataset::Save(boost::shared_ptr<cv::Mat> descriptors) {
  std::string file("dataset_"+output_+".yml");
  cv::FileStorage fs(file, cv::FileStorage::WRITE);
  fs << "database" << *descriptors;
  fs.release();
}

void Dataset::Load(boost::shared_ptr<cv::Mat> descriptors,
                   boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > poses) {
  double time_start = pcl::getTime();
  std::string file("dataset_"+output_+".yml");
  std::cout << file << std::endl;
  cv::FileStorage fs(file, cv::FileStorage::READ);
  fs["database"] >> (*descriptors);
  fs.release();

  poses->resize(descriptors->rows, 3);

  std::string line, label;
  std::ifstream file_stream(path_+poses_);
  for (int i = 0; i < descriptors->rows; i++) {
    double dummy, tx, ty, tz;
    std::getline(file_stream, line);
    std::stringstream stream(line);
    if (format_ == "kitti") {
      stream >> dummy >> dummy >> dummy >> tx
             >> dummy >> dummy >> dummy >> ty
             >> dummy >> dummy >> dummy >> tz;
    } else if (format_ == "freiburg") {
      stream >> dummy >> tx >> tz >> ty;
    } else if (format_ == "malaga") {
      stream >> dummy >> dummy >> dummy >> dummy
             >> dummy >> dummy >> dummy >> dummy
             >> tx >> ty >> tz;
    }
    (*poses)(i, 0) = tx;
    (*poses)(i, 1) = ty;
    (*poses)(i, 2) = tz;
  }
  file_stream.close();

  std::cout << "Database load took "
            << double(pcl::getTime() - time_start)
            << std::endl;
}

pcl::PointCloud<PointType>::Ptr Dataset::Frame(int n) {
  double time_start = pcl::getTime();
  pcl::PCDReader reader;
  pcl::PointCloud<PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointType> >();
  std::string path = path_;
  path.append(frames_);
  if (mask_ > 0) path.append(Util::ZeroLeftPadding(mask_, n));
  else path.append(std::to_string(n));
  path.append(".pcd");
  Util::AssertFileExists(path);
  reader.read(path, *cloud);
  cloud->header.seq = n;
  PointCloudHelper::RemoveNaN(cloud);
  std::cout << "Cloud load took "
            << double(pcl::getTime() - time_start)
            << std::endl;

  if (downsample_ > 0) {
    pcl::PointCloud<PointType>::Ptr downsampled;
    if (n == 0) resolution_ = PointCloudHelper::Resolution(cloud) * downsample_;
    downsampled = PointCloudHelper::Sample(cloud, resolution_);
    downsampled->header.seq = n;
    return downsampled;
  }

  return cloud;
}

void Dataset::path(std::string path) {
  path_ = path;
}

void Dataset::amount(int amount) {
  amount_ = amount;
}

void Dataset::frames(std::string frames) {
  frames_ = frames;
}

void Dataset::poses(std::string poses) {
  poses_ = poses;
}

void Dataset::timestamps(std::string timestamps) {
  timestamps_ = timestamps;
}

void Dataset::output(std::string output) {
  output_ = output;
}

void Dataset::format(std::string format) {
  format_ = format;
}

void Dataset::distance(int distance) {
  distance_ = distance;
}

std::string Dataset::output() {
  return output_;
}

int Dataset::distance() {
  return distance_;
}

int Dataset::amount() {
  return amount_;
}

void Dataset::mask(int mask) {
  mask_ = mask;
}

void Dataset::downsample(int downsample) {
  downsample_ = downsample;
}

} // namespace prototype2
