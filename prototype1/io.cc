//
// Created by phi on 20/03/18.
//

#include "io.h"

namespace prototype1 {

template<class T>
IO<T>::IO(std::string configFile) {
  this->LoadConfigFile(configFile);
}

template<class T>
void IO<T>::LoadConfigFile(std::string file) {
  this->AssertFileExists(file);

  std::ifstream infile(file);
  std::istringstream iss;
  std::string line, label;

  std::getline(infile, line);
  std::getline(infile, line);
  iss.str(line);
  iss >> label >> this->config_.dataset.width;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.dataset.height;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.dataset.size;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.dataset.path;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.dataset.reference;

  std::getline(infile, line);
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.normal_estimation.max_depth_change_factor;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.normal_estimation.normal_smoothing_size;

  std::getline(infile, line);
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.omp_segmentation.active;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.omp_segmentation.min_inliers;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.omp_segmentation.angular_threshold;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.omp_segmentation.distance_threshold;

  std::getline(infile, line);
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.ahc_segmentation.active;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.ahc_segmentation.refinement;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.ahc_segmentation.min_support;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.ahc_segmentation.window_width;
  std::getline(infile, line);
  iss.clear();
  iss.str(line);
  iss >> label >> this->config_.ahc_segmentation.window_height;

  infile.close();
}

template<class T>
void IO<T>::AssertFileExists(std::string file) {
  if (!boost::filesystem::exists(file)) {
    file.append(" not found!");
    std::cerr << file << std::endl;
    throw std::exception();
  }
}

template<class T>
void IO<T>::LoadPointCloud(typename pcl::PointCloud<T>::Ptr pointcloud, int i) {
  std::string cloudfile = std::to_string(i);
  cloudfile = std::string((5 - (int) ((i < 10) ? 1 : log10(i) + 1)), '0').append(cloudfile);
  cloudfile.append(".pcd");
  this->LoadPointCloud(pointcloud, cloudfile);
  this->LoadDatasetReference(i);
}

template<class T>
void IO<T>::LoadDatasetReference(int index) {
  std::string file = config_.dataset.path;
  file.append(config_.dataset.reference);
  this->AssertFileExists(file);

  std::ifstream infile(file);
  std::istringstream iss;
  std::string line, timestamp1, timestamp2, timestamp3
    , tx, ty, tz, qx, qy, qz, qw
    , reference_depth, reference_rgb;

  for (int i = 0; i <= index; i++)
    std::getline(infile, line);

  iss.str(line);
  iss >> timestamp1 >> tx >> ty >> tz
      >> qx >> qy >> qz >> qw
      >> timestamp2 >> reference_depth
      >> timestamp3 >> reference_rgb;
  infile.close();

  this->reference_rgb_ = this->config_.dataset.path;
  this->reference_rgb_.append(reference_rgb);
  this->reference_depth_ = this->config_.dataset.path;
  this->reference_depth_.append(reference_depth);

  //std::cout << " rd" << this->reference_depth_ << " rr" << this->reference_rgb_ << std::endl;
}

template<class T>
void IO<T>::LoadPointCloud(typename pcl::PointCloud<T>::Ptr pointcloud, std::string filename) {
  pcl::PCDReader reader;
  std::string file = this->config_.dataset.path;
  file.append(filename);

  this->AssertFileExists(file);

  reader.read(file, *pointcloud);
  std::cout << "Frame: " << filename
            << " " << pointcloud->width
            << "x" << pointcloud->height << std::endl;
}

template<class T>
void IO<T>::Show(std::string title, cv::Mat image) {
  cv::Mat rgb = cv::imread(reference_rgb_, CV_LOAD_IMAGE_COLOR);
  cv::Mat depth = cv::imread(reference_depth_, CV_LOAD_IMAGE_COLOR);

  if (image.cols != rgb.cols) {
    cv::resize(image, image, cv::Size(rgb.cols, rgb.rows));
  }

  cv::Mat output(cv::Size(rgb.cols*2, rgb.rows*2), image.type(), cv::Scalar::all(0));
  cv::Mat roi = output(cv::Rect(0, 0, rgb.cols, rgb.rows));
  rgb.copyTo(roi);
  roi = output(cv::Rect(rgb.cols, 0, rgb.cols, rgb.rows));
  depth.copyTo(roi);
  roi = output(cv::Rect(0, rgb.rows, rgb.cols, rgb.rows));
  image.copyTo(roi);

  cv::resize(output, output, cv::Size(800, 600));

  cv::imshow(title, output);
  cv::waitKey(1);
}

template class IO<pcl::PointXYZ>;
template class IO<pcl::PointXYZRGB>;

} // namespace prototype1
