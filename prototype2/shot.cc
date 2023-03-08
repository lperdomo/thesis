//
// Created by phi on 21/05/18.
//

#include "shot.h"

namespace prototype2 {

template<class T>
ShotEstimation<T>::ShotEstimation(pcl::search::KdTree<PointType>::Ptr kdtree) {
  method_.setSearchMethod(kdtree);
}

template<class T>
void ShotEstimation<T>::ExtractImpl(pcl::PointCloud<PointType>::ConstPtr cloud) {
  //double time_start = pcl::getTime();
  PointType centroid = PointCloudHelper::Centroid<PointType>(cloud);
  method_.setInputCloud(PointCloudHelper::MakePointCloud<PointType>(centroid));
  method_.setRadiusSearch(PointCloudHelper::MaxDistance(centroid, cloud));
  pcl::PointCloud<typename T::Type> result;
  method_.setInputNormals(normal_->cloud());
  method_.setSearchSurface(cloud);
  method_.compute(result);
  result_ = cv::Mat(result.width, T::Type::descriptorSize(), CV_32F);
  for (int j = 0; j < result_.rows; j++) {
    for (int i = 0; i < result_.cols; i++) {
      result_.at<float>(j, i) = result.points[j].descriptor[i];
    }
  }
  //std::cout << "Shot estimation took " << double(pcl::getTime() - time_start) << std::endl;
}

} // namespace prototype2
