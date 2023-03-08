//
// Created by phi on 16/05/18.
//

#ifndef PROTOTYPE2_OUTLIER_H
#define PROTOTYPE2_OUTLIER_H

#include "util.h"

namespace prototype2 {

class OutlierBase {
public:
  typedef boost::shared_ptr<OutlierBase> Ptr;
  virtual std::vector<std::vector<cv::DMatch> > Reject(std::vector<std::vector<cv::DMatch> > matches);
};

class LoweRatioOutlier {
public:
  typedef boost::shared_ptr<LoweRatioOutlier> Ptr;
  std::vector<std::vector<cv::DMatch> > Reject(std::vector<std::vector<cv::DMatch> > matches);
private:
  float ratio_;
};

} // namespace prototype2

#endif //PROTOTYPE2_OUTLIER_H
