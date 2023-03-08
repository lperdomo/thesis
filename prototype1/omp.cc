//
// Created by phi on 28/03/18.
//

#include "omp.h"

namespace prototype1 {
namespace segmentation {

template<class T>
OrganizedMultiPlane<T>::OrganizedMultiPlane(unsigned int min_inliers
    , double angular_threshold
    , double distance_threshold
    , typename prototype1::Feature<T>::Ptr feature) {
  segmentation_.setMinInliers(min_inliers);
  segmentation_.setAngularThreshold(angular_threshold);
  segmentation_.setDistanceThreshold(distance_threshold);
  feature_ = feature;
  labels_.reset(new pcl::PointCloud<pcl::Label>);
};

template<class T>
OrganizedMultiPlane<T>::OrganizedMultiPlane(typename IO<T>::OmpSegmentation param
    , typename prototype1::Feature<T>::Ptr feature) {
  segmentation_.setMinInliers(param.min_inliers);
  segmentation_.setAngularThreshold(param.angular_threshold);
  segmentation_.setDistanceThreshold(param.distance_threshold);
  feature_ = feature;
  labels_.reset(new pcl::PointCloud<pcl::Label>);
};

template<class T>
void OrganizedMultiPlane<T>
::Compute(typename pcl::PointCloud<T>::Ptr pointcloud
    , pcl::PointCloud<pcl::Normal>::Ptr normalcloud) {
  double segmentation_start = pcl::getTime();
  this->Clear();
  segmentation_.setInputNormals(normalcloud);
  segmentation_.setInputCloud(pointcloud);
  segmentation_.segmentAndRefine(regions_
      , model_coefficients_
      , inlier_indices_
      , labels_
      , label_indices_
      , boundary_indices_);
  double segmentation_end = pcl::getTime();
  std::cout << "OrganizedMultiPlaneSegmentation took "
            << double(segmentation_end - segmentation_start) << "s "
            << std::endl
            << "Detected planes: " << regions_.size()
            << std::endl;
}

template<class T>
void OrganizedMultiPlane<T>
::Compute(typename pcl::PointCloud<T>::Ptr pointcloud) {
  feature_->NormalEstimation(pointcloud);
  this->Compute(pointcloud, feature_->normalcloud_);
}

template<class T>
void OrganizedMultiPlane<T>::Clear() {
  labels_.reset(new pcl::PointCloud<pcl::Label>);
  model_coefficients_.clear();
  boundary_indices_.clear();
  inlier_indices_.clear();
  label_indices_.clear();
  regions_.clear();
}

template<class T>
cv::Mat OrganizedMultiPlane<T>::Result(typename pcl::PointCloud<T>::Ptr pointcloud) {
  std::vector<std::vector<int> > map;
  for (int y = 0; y < pointcloud->height; y++) {
    for (int x = 0; x < pointcloud->width; x++) {
      std::vector<int> p = {x, y};
      map.push_back(p);
    }
  }

  cv::Mat segmented_image = cv::Mat(pointcloud->height, pointcloud->width, CV_8UC3, cvScalar(0,0,0));
  for (size_t i = 0; i < regions_.size(); i++) {
    //std::mt19937 mt((int)i+50);
    srand((int)i+50);
    int r = rand() % (255 + 1 - 100) + 100
      , g = rand() % (255 + 1 - 100) + 100
      , b = rand() % (255 + 1 - 100) + 100;
    //std::uniform_int_distribution<int> r(100, 255), g(100, 255), b(100, 255);
    for (size_t j = 0; j < inlier_indices_[i].indices.size(); j++) {
      segmented_image.at<cv::Vec3b>(map[inlier_indices_[i].indices[j]][1]
          , map[inlier_indices_[i].indices[j]][0])[0] = b;//b(mt);
      segmented_image.at<cv::Vec3b>(map[inlier_indices_[i].indices[j]][1]
          , map[inlier_indices_[i].indices[j]][0])[1] = g;//g(mt);
      segmented_image.at<cv::Vec3b>(map[inlier_indices_[i].indices[j]][1]
          , map[inlier_indices_[i].indices[j]][0])[2] = r;//r(mt);
    }
  }

  return segmented_image;
}

template class OrganizedMultiPlane<pcl::PointXYZ>;
template class OrganizedMultiPlane<pcl::PointXYZRGB>;

} // namespace segmentation
} // namespace prototype1

