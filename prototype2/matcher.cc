//
// Created by phi on 16/05/18.
//

#include "matcher.h"

namespace prototype2 {

int   BruteforceMatcher::k         = 0;
int   BruteforceMatcher::window    = 0;
float BruteforceMatcher::threshold = 0;

int   Flann::k                = 0;
int   Flann::trees            = 0;
int   Flann::recursive_checks = 0;
int   Flann::window           = 0;
float Flann::threshold        = 0;

Matcher::Matcher() :
  match_lbound_(-1, -1, std::numeric_limits<float>::max()),
  match_ubound_(-1, -1, 0) {

}

std::vector<std::vector<std::vector<cv::DMatch> > >& Matcher::matches() {
  return matches_;
}

BruteforceMatcher::BruteforceMatcher() :
  Matcher::Matcher() {
}

void BruteforceMatcher::Train(boost::shared_ptr<cv::Mat> descriptors) {
  double time_start = pcl::getTime();
  std::vector<cv::Mat> descriptor;
  descriptor.resize(descriptors->rows);
  window_.resize(descriptors->rows);
  for (int i = 0; i < descriptors->rows; i++) {
    descriptor[i] = descriptors->row(i).clone();
    if (BruteforceMatcher::window > 0) window_(i) = 0;
    else window_(i) = 1;
  }
  method_.add(descriptor);
  method_.train();
  std::cout << "Database training took "
            << double(pcl::getTime() - time_start)
            << std::endl;
}

void BruteforceMatcher::KnnMatch(cv::Mat query) {
  std::vector<cv::DMatch> knn(BruteforceMatcher::k, cv::DMatch(-1, -1, -1, std::numeric_limits<float>::max()));
  int sequence_length = method_.getTrainDescriptors().size();

  if (matches_.empty()) matches_.resize(sequence_length);
  for (int i = 0; i < sequence_length; i++) {
    method_.knnMatch(query, method_.getTrainDescriptors()[i], matches_[i], 1);
    if (!matches_[i].empty()) {
	    matches_[i][0][0].imgIdx = i;
      if (matches_[i][0][0].distance < match_lbound_.distance) match_lbound_ = matches_[i][0][0];
      if (matches_[i][0][0].distance > match_ubound_.distance
          && matches_[i][0][0].distance < std::numeric_limits<float>::max())
        match_ubound_ = matches_[i][0][0];
      if (window_(i) && (matches_[i][0][0].distance < knn[BruteforceMatcher::k-1].distance)) {
	      int k;
	      for (k = BruteforceMatcher::k-2; k >= 0 && knn[k].distance > matches_[i][0][0].distance; k--) {
		      knn[k+1] = knn[k];
		    }
	      knn[k+1] = matches_[i][0][0];
      }
    }
  }

  knn_.clear();
  for (int i = 0; i < BruteforceMatcher::k; i++) {
	  if (knn[i].imgIdx != -1) knn_.push_back(knn[i]);
  }
}

/*template<>
void Matcher<Bruteforce>::KnnWindowedMatch(cv::Mat query) {
  double time_start = pcl::getTime();
  int sequence_length = method_.getTrainDescriptors().size();
  if (matches_.empty()) matches_.resize(sequence_length);
  for (int i = 0; i < sequence_length; i++) {
    method_.knnMatch(query, method_.getTrainDescriptors()[i], matches_[i], Bruteforce::k);
    if (!matches_[i].empty()) {
      if (matches_[i][0][0].distance < match_lbound_.distance) match_lbound_ = matches_[i][0][0];
      if (matches_[i][0][0].distance > match_ubound_.distance
       && matches_[i][0][0].distance < std::numeric_limits<float>::max())
      match_ubound_ = matches_[i][0][0];
    }
  }
  std::cout << "Query matching took "
            << double(pcl::getTime() - time_start)
            << std::endl;
}

template<>
void Matcher<Flann>::KnnWindowedMatch(cv::Mat query) {
  double time_start = pcl::getTime();
  std::cout << "Query matching took "
            << double(pcl::getTime() - time_start)
            << std::endl;
}*/

float BruteforceMatcher::BestMatchDistanceNormalized(int current) {
  if (matches_[current][0][0].distance < std::numeric_limits<float>::max()) {
    return matches_[current][0][0].distance/match_ubound_.distance;
  }
  return 1;
  //return matches_[current][0][0].distance;
  //if (matches_[current][0][0].distance > 1) {
  //  std::cout << current << std::endl;
  //  std::cout << teste << std::endl;
  //  std::cout << matches_[current][0][0].distance << std::endl;
    //std::exit(0);
  //}
  //return (matches_[current][0][0].distance <= (1-T::threshold) ? matches_[current][0][0].distance : 1);
}

void BruteforceMatcher::Window(int current) {
//  for (int j = 0; j < 2; j++) {
//    for (current = 0; current < 100; current++) {
    int sequence_length = method_.getTrainDescriptors().size();
    if (BruteforceMatcher::window > 0) {
      int window = std::min(BruteforceMatcher::window, sequence_length - 1);
      int upper_bound = std::min(current + window, sequence_length - 1);
      int previous = std::max(current - window - 1, 0);

      if (current == 0) {
        //for (int i = size - window - 1; i < size; i++)
        //  mask_[i].at<uchar>(0, 0) = 1;
        for (int i = 1; i <= window; i++)
          window_(i) = 0;
      }
      if (current > window) {
		    window_(previous) = 1;
        if (upper_bound != sequence_length) window_(upper_bound) = 0;
      } else  window_(upper_bound) = 0;
    }
//      for (int p = 0; p < mask_.size(); p++) {
//        std::cout << p << mask_[p];
//      }
//      std::cout << std::endl;
//      std::cin.get();
//    }
//  }
}

int BruteforceMatcher::Result() {
  if (!knn_.empty()) {
    float threshold = (1-BruteforceMatcher::threshold) * match_ubound_.distance;
    //std::cout << "T::threshold " << T::threshold << std::endl;
    //std::cout << "match_ubound_.distance " << match_ubound_.distance << std::endl;
    //std::cout << "threshold " << threshold << std::endl;
    //std::cout << "knn_ " << knn_.size() << std::endl;
    for (int i = 0; i < knn_.size(); i++) {
	    //std::cout << "knn_[i].distance " << knn_[i].distance << std::endl;
	    //std::cout << "knn_[i].imgIdx " << knn_[i].imgIdx << std::endl;
	    //std::cout << "knn_[i].queryIdx " << knn_[i].queryIdx << std::endl;
	    //std::cout << "knn_[i].trainIdx " << knn_[i].trainIdx << std::endl;
	    if (knn_[i].distance <= threshold) return knn_[i].imgIdx;
	  }
  }
  return -1;
}

/*
template<typename T>
std::vector<int> Matcher<T>::Reselect() {
  if (matches_.empty()) return std::vector<int>();
  std::vector<int>   result(T::candidates, 0);
  std::vector<float> distance(T::candidates, std::numeric_limits<float>::max());
  float threshold = (1-T::threshold) * match_ubound_.distance;
  //for (int j = 0; j < matches_.size(); j++) std::cout << matches_[j][0][0].distance << std::endl;
  //Simple best descending results under a threshold using a window
  for (int i = 0; i < matches_.size(); i++) {
    if (!matches_[i].empty()) {
      //if (matches_[i][0][0].distance < distance[K-1]) {
      //if (window_(i) && matches_[i][0][0].distance < distance[T::candidates-1]
      if (matches_[i][0][0].distance < distance[T::candidates-1]
          && matches_[i][0][0].distance <= threshold) {
        //int k = T::candidates-2;
        int k;
        //std::cout << "k=" << k << std::endl;
        //std::cout << "result[k] > teste[i] = " << distance[k] << " > " << matches_[i][0][0].distance << std::endl;
        for (k = T::candidates-2; k >= 0 && distance[k] > matches_[i][0][0].distance; k--) {
          result[k+1]   = result[k];
          distance[k+1] = distance[k];
        }
        result[k+1]   = i;
        distance[k+1] = matches_[i][0][0].distance;
        //for (int j = 0; j < K; j++) std::cout << result[j] << std::endl;
        //std::cout << "-----" << std::endl;
      }
    }
  }

  //std::cout << "result size " << result.size() << std::endl;
  //for (int i = 0; i < result.size(); i++) {
  //  std::cout << result[i] << std::endl;
  //}
  //std::exit(0);
  return result;
}
*/
} // namespace prototype2
