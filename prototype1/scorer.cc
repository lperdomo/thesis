//
// Created by phi on 16/05/18.
//

#include "scorer.h"

namespace prototype2 {

double Scorer::threshold = 0;

void NaiveScoreEstimation::Setup(int candidate_amount) {
  candidates_.resize(candidate_amount);
}

void NaiveScoreEstimation::Score(std::vector<std::vector<cv::DMatch> > matches) {
  double time_start = pcl::getTime();
  std::cout << "teste" << std::endl;
  std::fill(candidates_.begin(), candidates_.end(), 0);
  most_voted_amount_ = 0;
  most_voted_        = -1;
  total_votes_       = 0;
  std::cout << "matches.size() = " << matches.size() << std::endl;
  for (size_t i = 0; i < matches.size(); i++) {
    std::cout << "matches[i].size() = " << matches[i].size() << std::endl;
    for (size_t j = 0; j < matches[i].size(); j++) {
      std::cout << "candidato " << matches[i][j].imgIdx << std::endl;
      candidates_[matches[i][j].imgIdx]++;
      if (candidates_[matches[i][j].imgIdx] > most_voted_amount_) {
        most_voted_ = matches[i][j].imgIdx;
        most_voted_amount_ = candidates_[matches[i][j].imgIdx];
      }
      total_votes_++;
    }
  }
  std::cout << "resultado: candidato " << most_voted_ << std::endl;
  std::cout << "Scoring took "
            << double(pcl::getTime() - time_start)
            << std::endl;
}

int NaiveScoreEstimation::Result() {
  return most_voted_;
}

/*
template class Matcher<BruteForce>;
template class Matcher<Flann>;

template<class T>
Matcher<T>::Matcher(T method, int k) : method_(method), k_(k) {

}

template<class T>
void Matcher<T>::k(int k) {
  k_ = k;
}

template<class T>
int Matcher<T>::k() {
  return k_;
}

template<class T>
void Matcher<T>::Train(std::vector<cv::Mat> database) {
  double time_start = pcl::getTime();
  method_.add(database);
  method_.train();
  std::cout << "Database training took "
            << double(pcl::getTime() - time_start)
            << std::endl;
}

template<class T>
std::vector<std::vector<cv::DMatch> > Matcher<T>::Match(cv::Mat query) {
  std::vector<std::vector<cv::DMatch> > match;
  double time_start = pcl::getTime();
  method_.knnMatch(query, match, k_);
  std::cout << "Query matching took "
            << double(pcl::getTime() - time_start)
            << std::endl;
  return match;
}

/*
Match::Match() {

}

void Match::SetCandidates(int amount) {
  scores_.candidates.resize(amount);
}

void Match::ResetScores() {
  std::fill(scores_.candidates.begin(), scores_.candidates.end(), 0);
  scores_.max_voted_candidate = 0;
  scores_.max_votes           = 0;
  scores_.total               = 0;
}

void Match::ScoreMatchNaively(cv::DMatch match) {
  scores_.candidates[match.imgIdx]++;
  if (scores_.candidates[match.imgIdx] > scores_.max_votes) {
    scores_.max_voted_candidate = match.imgIdx;
    scores_.max_votes = scores_.candidates[match.imgIdx];
  }
  scores_.total++;
}
*/
} // namespace prototype2
