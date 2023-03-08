//
// Created by phi on 19/07/18.
//

#include "pch.h"

namespace prototype2 {

template<class T>
typename pcl::PointCloud<T>::Ptr PointCloudHelper::MakePointCloud(T point) {
  typename pcl::PointCloud<T>::Ptr cloud(new typename pcl::PointCloud<T>);
  cloud->points.push_back(point);
  return cloud;
}

template<class T>
typename pcl::PointCloud<T>::Ptr PointCloudHelper::Filter(typename pcl::PointCloud<T>::ConstPtr cloud,
                                                          pcl::PointIndices::ConstPtr indices, bool filter_out) {
  typename pcl::ExtractIndices<T> extractor;
  typename pcl::PointCloud<T>::Ptr filtered(new typename pcl::PointCloud<T>);
  extractor.setInputCloud(cloud);
  extractor.setIndices(indices);
  extractor.setNegative(!filter_out);
  extractor.filter(*filtered);
  return filtered;
}

template<class T>
T PointCloudHelper::Centroid(typename pcl::PointCloud<T>::ConstPtr cloud) {
  T centroid;
  pcl::computeCentroid<T, T>(*cloud, centroid);
  return centroid;
}

pcl::PointCloud<PointType>::Ptr PointCloudHelper::Sample(pcl::PointCloud<PointType>::ConstPtr cloud, float leaf_size) {
  pcl::VoxelGrid<PointType> sampler;
  pcl::PointCloud<PointType>::Ptr sampled_cloud = boost::make_shared<pcl::PointCloud<PointType> >();
  sampler.setInputCloud(cloud);
  sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
  sampler.filter(*sampled_cloud);
  std::cout << cloud->header.seq << std::endl;
  std::string teste = std::to_string(cloud->header.seq);
  teste.append(".pcd");
  pcl::io::savePCDFileASCII(teste, *sampled_cloud);
  return sampled_cloud;
}

double PointCloudHelper::Resolution(typename pcl::PointCloud<PointType>::ConstPtr cloud) {
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud(cloud);
  for (size_t i = 0; i < cloud->size(); i++) {
    if (!pcl::isFinite(cloud->points[i])) {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2) {
      res += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0) {
    res /= n_points;
  }
  return res;
}

pcl::PointIndices::Ptr PointCloudHelper::RemoveNaN(pcl::PointCloud<pcl::Normal>::Ptr cloud) {
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices->indices);
  return indices;
}

pcl::PointIndices::Ptr PointCloudHelper::RemoveNaN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices->indices);
  return indices;
}

pcl::PointIndices::Ptr PointCloudHelper::RemoveNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices->indices);
  return indices;
}


float PointCloudHelper::MaxDistance(PointType point, pcl::PointCloud<PointType>::ConstPtr cloud) {
  Eigen::Vector4f max_eigen_vector;
  PointType max_point;
  pcl::getMaxDistance(*cloud, point.getVector4fMap(), max_eigen_vector);
  //max_eigen_vector[3] = 0;
  max_point.getVector4fMap() = max_eigen_vector;
  return pcl::euclideanDistance<PointType>(point, max_point);
}

template pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::MakePointCloud<pcl::PointXYZRGB>(pcl::PointXYZRGB point);
template pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudHelper::MakePointCloud<pcl::PointXYZ>(pcl::PointXYZ point);
template pcl::PointCloud<pcl::Normal>::Ptr PointCloudHelper::MakePointCloud<pcl::Normal>(pcl::Normal point);
template pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::Filter<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointIndices::ConstPtr indices, bool filter_out);
template pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudHelper::Filter<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::ConstPtr indices, bool filter_out);
template pcl::PointCloud<pcl::Normal>::Ptr PointCloudHelper::Filter<pcl::Normal>(pcl::PointCloud<pcl::Normal>::ConstPtr cloud, pcl::PointIndices::ConstPtr indices, bool filter_out);
template pcl::PointXYZRGB PointCloudHelper::Centroid<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
template pcl::PointXYZ PointCloudHelper::Centroid<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
template pcl::Normal PointCloudHelper::Centroid<pcl::Normal>(pcl::PointCloud<pcl::Normal>::ConstPtr cloud);
} // namespace prototype
