//
// Created by phi on 16/05/18.
//

#include "outlier.h"

namespace prototype2 {

std::vector<std::vector<cv::DMatch> > LoweRatioOutlier::Reject(std::vector<std::vector<cv::DMatch> > matches) {
  double time_start = pcl::getTime();
  std::vector<std::vector<cv::DMatch> > inliers;
  inliers.resize(matches.size());
  for (size_t i = 0; i < matches.size(); i++) {
    double k_closest = matches[i][0].distance;
    for (size_t j = 0; j < matches[i].size(); j++) {
      //reject delta_dk/delta_di <= threshold_d
      //std::cout << "i" << i << "-j" << j << " distance=" << matches_aux[i][j].distance << " closest=" << k_closest << " ratio=" << k_closest/matches_aux[i][j].distance << std::endl;
      if (k_closest/matches[i][j].distance <= ratio_) {
        inliers[i].push_back(matches[i][j]);
      }
    }
  }
  std::cout << "Outlier rejection took "
            << double(pcl::getTime() - time_start)
            << std::endl;
  return inliers;
}

} // namespace prototype2
