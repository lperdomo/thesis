//
// Created by phi on 21/05/18.
//

#include "cospair.h"

namespace prototype2 {
/*
template class CospairEstimation<PointType>;

template<class T>
void CospairEstimation<T>::input_cloud(typename pcl::PointCloud<T>::ConstPtr input_cloud) {
  input_cloud_ = input_cloud;
}

template<class T>
void CospairEstimation<T>::input_normals(pcl::PointCloud<pcl::Normal>::ConstPtr input_normals) {
  input_normals_ = input_normals;
}

template<class T>
void CospairEstimation<T>::search_surface(typename pcl::PointCloud<T>::ConstPtr search_surface) {
  search_surface_ = search_surface;
}
template<class T>
void CospairEstimation<T>::search_surface_normals(pcl::PointCloud<pcl::Normal>::ConstPtr search_surface_normals) {
  search_surface_normals_ = search_surface_normals;
}

template<class T>
void CospairEstimation<T>::radius(double radius) {
  radius_ = radius;
}

template<class T>
void CospairEstimation<T>::levels(int levels) {
  levels_ = levels;
}

template<class T>
void CospairEstimation<T>::depth_bins(int depth_bins) {
  depth_bins_ = depth_bins;
}

template<class T>
void CospairEstimation<T>::color_bins(int color_bins) {
  color_bins_ = color_bins;
}

template<class T>
void CospairEstimation<T>::color_scheme(int color_scheme) {
  if (color_scheme == 1) {
    color_scheme_ = ColorScheme::RGB;
  } else if (color_scheme == 2) {
    color_scheme_ = ColorScheme::HSV;
  } else if (color_scheme == 3) {
    color_scheme_ = ColorScheme::LAB;
  } else if (color_scheme == 4) {
    color_scheme_ = ColorScheme::H;
  }
}

template<class T>
void CospairEstimation<T>::color_scheme(ColorScheme color_scheme) {
  color_scheme_ = color_scheme;
}

template<class T>
void CospairEstimation<T>::color_l1(bool color_l1) {
  color_l1_ = color_l1;
}

template<class T>
int CospairEstimation<T>::depth_feature_level_dimension() {
  return depth_feature_level_dimension_;
}

template<class T>
int CospairEstimation<T>::color_feature_level_dimension() {
  return color_feature_level_dimension_;
}

template<class T>
int CospairEstimation<T>::dimension() {
  return (levels_ * depth_feature_level_dimension_) + (levels_ * color_feature_level_dimension_);
}

template<class T>
void CospairEstimation<T>::Compute(pcl::PointCloud<CospairDescriptor> &descriptors) {
  if (color_scheme_ == ColorScheme::RGB) {
    this->ComputeDescriptor<pcl::PointXYZRGB>(descriptors);
  } else if (color_scheme_ == ColorScheme::HSV) {
    this->ComputeDescriptor<pcl::PointXYZHSV>(descriptors);
  } else if (color_scheme_ == ColorScheme::H) {
    this->ComputeDescriptor<pcl::PointXYZH>(descriptors);
  }
}

template<class T>
void CospairEstimation<T>::Dimensionate() {
  depth_feature_level_dimension_ = depth_bins_ * 3;
  if (color_scheme_ == ColorScheme::RGB) {
    color_feature_level_dimension_ = color_bins_ * 3;
  } else if (color_scheme_ == ColorScheme::HSV) {
    color_feature_level_dimension_ = color_bins_ * 3;
  } else if (color_scheme_ == ColorScheme::H) {
    color_feature_level_dimension_ = color_bins_ * 1;
  }
}

template<class T>
template<class T2>
void CospairEstimation<T>::ComputeDescriptor(pcl::PointCloud<CospairDescriptor> &descriptors) {
  typename pcl::search::KdTree<T>::Ptr tree(new typename pcl::search::KdTree<T>);
  tree->setInputCloud(search_surface_);
  int levelsize_total = depth_feature_level_dimension_ + color_feature_level_dimension_;
  int histsize_total  = this->dimension();
  int levelsearch[levels_];
  //Calculate SPAIR and push to SPFH features
  for (int j = 0; j < input_cloud_->points.size(); j++) {
    if (pcl::isFinite(input_normals_->points[j]) && pcl::isFinite(input_cloud_->points[j])) {
      CospairDescriptor spair_features;
      spair_features.descriptor.resize(histsize_total, 0);
      std::vector<float> pointRadiusSquaredDistance;
      std::vector<int> pointIdxRadiusSearch;
      float deg_f1, deg_f2, deg_f3;
      int bin_f1, bin_f2, bin_f3;
      float f1, f2, f3, f4;
      int searchstart = 1;

      for (int l = 1; l <= levels_; l++) {
        unsigned int levelpaircount = 0;
        float level_features[depth_feature_level_dimension_];
        float level_features_rgb[color_feature_level_dimension_];
        int searchsize;
        double r;

        for (int z = 0; z < depth_feature_level_dimension_; z++)
          level_features[z] = 0.0;
        for (int z = 0; z < color_feature_level_dimension_; z++)
          level_features_rgb[z] = 0.0;

        r = ((l * 1.0) / levels_) * radius_;
        //Number of points inside radius r
        searchsize = tree->radiusSearch(input_cloud_->points[j], r, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0);

        levelsearch[l - 1] = searchsize;
        if (l != 1)
          searchstart = levelsearch[l - 2];

        // Iterate over all the points in the level neighborhood
        for (int i_idx = searchstart; i_idx < searchsize; ++i_idx) {
          // If the 3D points are invalid, don't bother estimating, just continue
          if (pcl::isFinite(search_surface_->points[pointIdxRadiusSearch[i_idx]])
              && pcl::isFinite(search_surface_normals_->points[pointIdxRadiusSearch[i_idx]])) {
            int p1;
            p1 = pointIdxRadiusSearch[i_idx];
            levelpaircount++;
            //Calculate the pair features (angles)
            pcl::computePairFeatures(input_cloud_->points[j].getVector4fMap(),
                                     input_normals_->points[j].getNormalVector4fMap(),
                                     search_surface_->points[p1].getVector4fMap(),
                                     search_surface_normals_->points[p1].getNormalVector4fMap(),
                                     f1, f2, f3, f4);

            deg_f1 = pcl::rad2deg(f1) + 180;
            deg_f2 = pcl::rad2deg(Util::acos(f2));
            deg_f3 = pcl::rad2deg(Util::acos(f3));
            bin_f1 = int(floor(deg_f1 / (360.0 / depth_bins_)));
            bin_f2 = int(floor(deg_f2 / (180.0 / depth_bins_)));
            bin_f3 = int(floor(deg_f3 / (180.0 / depth_bins_)));

            level_features[bin_f1] += 1.0f;
            level_features[1 * depth_bins_ + bin_f2] += 1.0f;
            level_features[2 * depth_bins_ + bin_f3] += 1.0f;

            this->ComputeColorBins<T2>(search_surface_->points[p1], input_cloud_->points[j], level_features_rgb);
          }
        }
        //histograms are normalized using
        //the number of distinct points in each level
        if (levelpaircount != 0) {
          for (int n = 0; n < depth_feature_level_dimension_; n++) {
            level_features[n] = (level_features[n] / levelpaircount) * l;//(levels - (l-1));
            spair_features.descriptor[(l - 1) * levelsize_total + n] = level_features[n];
          }
        }
        if (levelpaircount != 0) {
          for (int n = 0; n < color_feature_level_dimension_; n++) {
            level_features_rgb[n] = (level_features_rgb[n] / levelpaircount) * l;// (levels - (l-1));
            spair_features.descriptor[(l - 1) * levelsize_total + depth_feature_level_dimension_ + n] = level_features_rgb[n];
          }
        }
      }
      descriptors.points.push_back(spair_features);
    }
  }
  descriptors.width = static_cast<uint32_t>(descriptors.points.size());
  descriptors.height = 1;
}

template<>
template<>
void CospairEstimation<PointType>::ComputeColorBins<pcl::PointXYZRGB>(PointType p1, PointType p2, float *color_features) {
  double r, g, b;
  int bin_r, bin_g, bin_b;

  if (color_l1_) {
    r = fabs(p1.r-p2.r);
    g = fabs(p1.g-p2.g);
    b = fabs(p1.b-p2.b);
  } else {
    r = p1.r;
    g = p1.g;
    b = p1.b;
  }

  bin_r = int(floor(r / (1.0/color_bins_)));
  bin_g = int(floor(g / (1.0/color_bins_)));
  bin_b = int(floor(b / (1.0/color_bins_)));
  color_features[bin_r] += 1.0;
  color_features[1 * color_bins_ + bin_g] += 1.0;
  color_features[2 * color_bins_ + bin_b] += 1.0;
}

template<>
template<>
void CospairEstimation<PointType>::ComputeColorBins<pcl::PointXYZHSV>(PointType p1, PointType p2, float *color_features) {
  pcl::PointXYZHSV pa, pb;
  double h, s, v;
  int bin_h, bin_s, bin_v;

  pcl::PointXYZRGBtoXYZHSV(p1, pa);

  if (color_l1_) {
    pcl::PointXYZRGBtoXYZHSV(p2, pb);
    h = fabs(pa.h-pb.h);
    s = fabs(pa.s-pb.s);
    v = fabs(pa.v-pb.v);
  } else {
    h = pa.h;
    s = pa.s;
    v = pa.v;
  }
  std::cout << "qwqwqwqwq" << std::endl;
  bin_h = int(floor((h/360) / (1.0/color_bins_)));
  bin_s = int(floor(s / (1.0/color_bins_)));
  bin_v = int(floor(v / (1.0/color_bins_)));
  color_features[bin_h] += 1.0;
  color_features[1 * color_bins_ + bin_s] += 1.0;
  color_features[2 * color_bins_ + bin_v] += 1.0;
  std::cout << "ffffffff" << std::endl;
}

template<>
template<>
void CospairEstimation<PointType>::ComputeColorBins<pcl::PointXYZH>(PointType p1, PointType p2, float *color_features) {
  pcl::PointXYZHSV pa, pb;
  double h;
  int bin_h;

  pcl::PointXYZRGBtoXYZHSV(p1, pa);

  if (color_l1_) {
    pcl::PointXYZRGBtoXYZHSV(p2, pb);
    h = fabs(pa.h-pb.h);
  } else {
    h = pa.h;
  }

  double u = 360/color_bins_;

  bin_h = int(floor((Util::ConstrainAngle(h+u)/360) / (1.0/color_bins_)));
  color_features[bin_h] += 1.0;
}

template void CospairEstimation<PointType>::ComputeDescriptor<pcl::PointXYZRGB>(pcl::PointCloud<prototype2::CospairDescriptor> &descriptors);
template void CospairEstimation<PointType>::ComputeDescriptor<pcl::PointXYZHSV>(pcl::PointCloud<prototype2::CospairDescriptor> &descriptors);
template void CospairEstimation<PointType>::ComputeDescriptor<pcl::PointXYZH>(pcl::PointCloud<prototype2::CospairDescriptor> &descriptors);
template void CospairEstimation<PointType>::ComputeColorBins<pcl::PointXYZRGB>(PointType p1, PointType p2, float *color_features);
template void CospairEstimation<PointType>::ComputeColorBins<pcl::PointXYZHSV>(PointType p1, PointType p2, float *color_features);
template void CospairEstimation<PointType>::ComputeColorBins<pcl::PointXYZH>(PointType p1, PointType p2, float *color_features);
*/

/*
template<class T>
Cospair<T>::Cospair(typename io::Configuration::CospairFeature params) {
  radius_     = params.radius;
  levels_     = params.levels;
  depth_bins_ = params.depth_bins;
  color_bins_ = params.color_bins;
}

template<>
void Cospair<pcl::PointXYZ>::DescriptorExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
}

template<>
void Cospair<pcl::PointXYZRGB>::DescriptorExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  double time_start = pcl::getTime();
  this->descriptor_cloud_.reset(new pcl::PointCloud<CospairDescriptor>);
  int histsize        = (levels_ * depth_bins_ * 3);
  int histsize_rgb    = (levels_ * color_bins_ * 3);
  int levelsize       = depth_bins_ * 3;
  int levelsize_rgb   = color_bins_ * 3;
  int levelsize_total = levelsize + levelsize_rgb;
  int histsize_total  = histsize + histsize_rgb;
  int levelsearch[levels_];

  //For each keypoint calculate SPAIR and push to SPFH features
  for (int j = 0; j < this->keypoint_cloud_->points.size(); j++) {
    int keypoint_indice = this->keypoint_indices_->indices[j];
    if (pcl::isFinite(this->normals_cloud_->points[keypoint_indice])
        && pcl::isFinite(cloud->points[keypoint_indice])) {
      //std::vector<float> spair_features(histsize_total, 0);
      CospairDescriptor spair_features;
      spair_features.descriptor.resize(histsize_total, 0);
      std::vector<float> pointRadiusSquaredDistance;
      std::vector<int>   pointIdxRadiusSearch;
      float deg_f1, deg_f2, deg_f3;
      int bin_f1, bin_f2, bin_f3;
      float f1, f2, f3, f4;
      int searchstart = 1;

      for (int l = 1; l <= levels_; l++) {
        //std::vector<float> level_features(levelsize,0);
        unsigned int levelpaircount = 0;
        float level_features[levelsize];
        float level_features_rgb[levelsize_rgb];
        int searchsize;
        double r;

        for (int z = 0; z < levelsize; z++)
          level_features[z] = 0.0;
        for (int z = 0; z < levelsize_rgb; z++)
          level_features_rgb[z] = 0.0;

        r = ((l * 1.0) / levels_) * radius_;

        //Number of points inside radius r

        searchsize = kdtree->radiusSearch(*cloud, keypoint_indice, r, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0);

        //std::cout<<j<<"; in level "<<l<<" "<<searchsize<<std::endl;
        levelsearch[l - 1] = searchsize;
        if (l != 1)
          searchstart = levelsearch[l - 2];

        // Iterate over all the points in the neighborhood
        for (int i_idx = searchstart; i_idx < searchsize; ++i_idx) {

          // If the 3D points are invalid, don't bother estimating, just continue
          if (pcl::isFinite(cloud->points[pointIdxRadiusSearch[i_idx]])
              && pcl::isFinite(this->normals_cloud_->points[pointIdxRadiusSearch[i_idx]])) {
            int p1, p2;
            p1 = pointIdxRadiusSearch[i_idx];
            p2 = keypoint_indice;

            levelpaircount++;

            //Calculate the pair features (angles)
            pcl::computePairFeatures(cloud->points[p2].getVector4fMap(),
                                     this->normals_cloud_->points[p2].getNormalVector4fMap(),
                                     cloud->points[p1].getVector4fMap(),
                                     this->normals_cloud_->points[p1].getNormalVector4fMap(),
                                     f1, f2, f3, f4);

            deg_f1 = pcl::rad2deg(f1) + 180;
            deg_f2 = pcl::rad2deg(Util::acos(f2));
            deg_f3 = pcl::rad2deg(Util::acos(f3));
            bin_f1 = int(floor(deg_f1 / (360.0 / depth_bins_)));
            bin_f2 = int(floor(deg_f2 / (180.0 / depth_bins_)));
            bin_f3 = int(floor(deg_f3 / (180.0 / depth_bins_)));

            level_features[bin_f1] = level_features[bin_f1] + 1.0f;
            level_features[1 * depth_bins_ + bin_f2] = level_features[1 * depth_bins_ + bin_f2] + 1.0f;
            level_features[2 * depth_bins_ + bin_f3] = level_features[2 * depth_bins_ + bin_f3] + 1.0f;

            //RGB-COLOR features
            int bin_r, bin_g, bin_b;
            float r, g, b;
            r = cloud->points[p1].r;
            g = cloud->points[p1].g;
            b = cloud->points[p1].b;

            bin_r = int(floor(r / (255.0 / color_bins_)));
            bin_g = int(floor(g / (255.0 / color_bins_)));
            bin_b = int(floor(b / (255.0 / color_bins_)));
            //std::cout<<bin_r<<" "<<bin_g<<" "<<bin_b<<std::endl;
            level_features_rgb[bin_r] = level_features_rgb[bin_r] + 1.0f;
            level_features_rgb[1 * color_bins_ + bin_g] = level_features_rgb[1 * color_bins_ + bin_g] + 1.0f;
            level_features_rgb[2 * color_bins_ + bin_b] = level_features_rgb[2 * color_bins_ + bin_b] + 1.0f;
          }

        }

        //std::cout<<levelpaircount<<" pairs in level"<<std::endl;
        if (levelpaircount != 0) {
          for (int n = 0; n < levelsize; n++) {
            //std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
            level_features[n] = (level_features[n] / levelpaircount) * l;//(levels - (l-1));
            //std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
            spair_features.descriptor[(l - 1) * levelsize_total + n] = level_features[n];
          }
        }

        if (levelpaircount != 0) {
          for (int n = 0; n < levelsize_rgb; n++) {
            //std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
            level_features_rgb[n] = (level_features_rgb[n] / levelpaircount) * l;// (levels - (l-1));
            //std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
            spair_features.descriptor[(l - 1) * levelsize_total + levelsize + n] = level_features_rgb[n];
          }
        }
      }
      this->descriptor_cloud_->push_back(spair_features);
    }
  }

  cv::Mat descriptors_temp(this->descriptor_cloud_->width, histsize_total, CV_32F);
  for (int j = 0; j < this->descriptor_cloud_->width; j++) {
    for (int i = 0; i < histsize_total; i++) {
      descriptors_temp.at<float>(j, i) = this->descriptor_cloud_->points[j].descriptor[i];
    }
  }
  this->descriptors_ = descriptors_temp;
  double time_end = pcl::getTime();
  std::cout << "CoSPAIRExtraction took "
            << double(time_end - time_start)
            << std::endl;
}

GLOBAL
 template<>
void GlobalCospair<pcl::PointXYZRGB>::DescriptorExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  double time_start = pcl::getTime();
  this->descriptor_cloud_.reset(new pcl::PointCloud<CospairDescriptor>);
  int histsize        = (levels_ * depth_bins_ * 3);
  int histsize_rgb    = (levels_ * color_bins_ * 3);
  int levelsize       = depth_bins_ * 3;
  int levelsize_rgb   = color_bins_ * 3;
  int levelsize_total = levelsize + levelsize_rgb;
  int histsize_total  = histsize + histsize_rgb;
  int levelsearch[levels_];


  pcl::PointXYZRGB centroid;
  pcl::computeCentroid<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloud, centroid);
  pcl::Normal normal_centroid;
  pcl::computeCentroid<pcl::Normal, pcl::Normal>(*this->normals_cloud_, normal_centroid);

  Eigen::Vector4f distance_max_point;
  pcl::getMaxDistance(*cloud, centroid.getVector4fMap(), distance_max_point);
  distance_max_point[3] = 0;
  radius_ = (centroid.getVector4fMap(), distance_max_point).norm();

  //Calculate SPAIR and push to SPFH features
  //int keypoint_indice = this->keypoint_indices_->indices[1];


  if (pcl::isFinite(normal_centroid)
      && pcl::isFinite(centroid)) {
    //std::vector<float> spair_features(histsize_total, 0);
    CospairDescriptor spair_features;
    spair_features.descriptor.resize(histsize_total, 0);
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<int>   pointIdxRadiusSearch;
    float deg_f1, deg_f2, deg_f3;
    int bin_f1, bin_f2, bin_f3;
    float f1, f2, f3, f4;
    int searchstart = 1;

    for (int l = 1; l <= levels_; l++) {
      //std::vector<float> level_features(levelsize,0);
      unsigned int levelpaircount = 0;
      float level_features[levelsize];
      float level_features_rgb[levelsize_rgb];
      int searchsize;
      double r;

      for (int z = 0; z < levelsize; z++)
        level_features[z] = 0.0;
      for (int z = 0; z < levelsize_rgb; z++)
        level_features_rgb[z] = 0.0;

      r = ((l * 1.0) / levels_) * radius_;

      //Number of points inside radius r

      //searchsize = kdtree->radiusSearch(*cloud, keypoint_indice, r, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0);
      kdtree->setInputCloud(cloud);
      searchsize = kdtree->radiusSearch(centroid, r, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0);

      //std::cout<<j<<"; in level "<<l<<" "<<searchsize<<std::endl;
      levelsearch[l - 1] = searchsize;
      if (l != 1)
        searchstart = levelsearch[l - 2];

      // Iterate over all the points in the neighborhood
      for (int i_idx = searchstart; i_idx < searchsize; ++i_idx) {

        // If the 3D points are invalid, don't bother estimating, just continue
        if (pcl::isFinite(cloud->points[pointIdxRadiusSearch[i_idx]])
            && pcl::isFinite(this->normals_cloud_->points[pointIdxRadiusSearch[i_idx]])) {
          int p1, p2;
          p1 = pointIdxRadiusSearch[i_idx];
          //p2 = keypoint_indice;

          levelpaircount++;

          //Calculate the pair features (angles)
          pcl::computePairFeatures(centroid.getVector4fMap(),
                                   normal_centroid.getNormalVector4fMap(),
                                   cloud->points[p1].getVector4fMap(),
                                   this->normals_cloud_->points[p1].getNormalVector4fMap(),
                                   f1, f2, f3, f4);

          deg_f1 = pcl::rad2deg(f1) + 180;
          deg_f2 = pcl::rad2deg(Util::acos(f2));
          deg_f3 = pcl::rad2deg(Util::acos(f3));
          bin_f1 = int(floor(deg_f1 / (360.0 / depth_bins_)));
          bin_f2 = int(floor(deg_f2 / (180.0 / depth_bins_)));
          bin_f3 = int(floor(deg_f3 / (180.0 / depth_bins_)));

          level_features[bin_f1] = level_features[bin_f1] + 1.0f;
          level_features[1 * depth_bins_ + bin_f2] = level_features[1 * depth_bins_ + bin_f2] + 1.0f;
          level_features[2 * depth_bins_ + bin_f3] = level_features[2 * depth_bins_ + bin_f3] + 1.0f;

          //RGB-COLOR features
          int bin_r, bin_g, bin_b;
          float r, g, b;
          r = cloud->points[p1].r;
          g = cloud->points[p1].g;
          b = cloud->points[p1].b;

          bin_r = int(floor(r / (255.0 / color_bins_)));
          bin_g = int(floor(g / (255.0 / color_bins_)));
          bin_b = int(floor(b / (255.0 / color_bins_)));
          //std::cout<<bin_r<<" "<<bin_g<<" "<<bin_b<<std::endl;
          level_features_rgb[bin_r] = level_features_rgb[bin_r] + 1.0f;
          level_features_rgb[1 * color_bins_ + bin_g] = level_features_rgb[1 * color_bins_ + bin_g] + 1.0f;
          level_features_rgb[2 * color_bins_ + bin_b] = level_features_rgb[2 * color_bins_ + bin_b] + 1.0f;
        }

      }

      //std::cout<<levelpaircount<<" pairs in level"<<std::endl;
      if (levelpaircount != 0) {
        for (int n = 0; n < levelsize; n++) {
          //std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
          level_features[n] = (level_features[n] / levelpaircount) * l;//(levels - (l-1));
          //std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
          spair_features.descriptor[(l - 1) * levelsize_total + n] = level_features[n];
        }
      }

      if (levelpaircount != 0) {
        for (int n = 0; n < levelsize_rgb; n++) {
          //std::cout<<level_features[n]<<" "<<level_features[n]/levelpaircount<<std::endl;
          level_features_rgb[n] = (level_features_rgb[n] / levelpaircount) * l;// (levels - (l-1));
          //std::cout<<(l-1)*levelsize + n<<" "<<level_features[n]<<std::endl;
          spair_features.descriptor[(l - 1) * levelsize_total + levelsize + n] = level_features_rgb[n];
        }
      }
    }
    this->descriptor_cloud_->push_back(spair_features);
    //std::cout << "aqui" << this->descriptor_cloud_->points.size() << std::endl;
  }

  cv::Mat descriptors_temp(this->descriptor_cloud_->width, histsize_total, CV_32F);
  for (int j = 0; j < this->descriptor_cloud_->width; j++) {
    for (int i = 0; i < histsize_total; i++) {
      descriptors_temp.at<float>(j, i) = this->descriptor_cloud_->points[j].descriptor[i];
    }
  }
  this->descriptors_ = descriptors_temp;
  double time_end = pcl::getTime();
  std::cout << "CoSPAIRExtraction took "
            << double(time_end - time_start)
            << std::endl;
}

*/

} // namespace prototype2
