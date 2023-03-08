//
// Created by phi on 16/05/18.
//

#ifndef PROTOTYPE2_MATCHER_H
#define PROTOTYPE2_MATCHER_H

#include <cvaux.h>
#include "util.h"

namespace prototype2 {

struct Flann {
  typedef cv::FlannBasedMatcher Method;
  static int k;
  static int trees;
  static int recursive_checks;
  static int window;
  static float threshold;
};

class Matcher {
public:
  typedef boost::shared_ptr<Matcher> Ptr;
  Matcher();
  virtual void Train(boost::shared_ptr<cv::Mat> descriptors) = 0;
  virtual void KnnMatch(cv::Mat query) = 0;
  //virtual void KnnWindowedMatch(cv::Mat query) = 0;
  virtual float BestMatchDistanceNormalized(int current) = 0;
  virtual void Window(int current) = 0;
  virtual int Result() = 0;
  std::vector<std::vector<std::vector<cv::DMatch> > >& matches();
  std::vector<cv::DMatch> knn_;
protected:
  std::vector<std::vector<std::vector<cv::DMatch> > > matches_;
  Eigen::Array<float, Eigen::Dynamic, 1> window_;
  cv::DMatch match_lbound_;
  cv::DMatch match_ubound_;
};

class BruteforceMatcher : public Matcher {
public:
  typedef boost::shared_ptr<BruteforceMatcher> Ptr;
  BruteforceMatcher();
  static int k;
  static int window;
  static float threshold;
  void Train(boost::shared_ptr<cv::Mat> descriptors) override;
  void KnnMatch(cv::Mat query) override;
  //void KnnWindowedMatch(cv::Mat query) override;
  float BestMatchDistanceNormalized(int current) override;
  void Window(int current) override;
  int Result() override;
private:
  cv::BFMatcher method_;
};

} // namespace prototype2

#endif //PROTOTYPE2_MATCHER_H
