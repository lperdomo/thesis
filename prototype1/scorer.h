//
// Created by phi on 16/05/18.
//

#ifndef PROTOTYPE2_SCORE_H
#define PROTOTYPE2_SCORE_H

#include "util.h"

namespace prototype2 {

class Scorer {
public:
  typedef boost::shared_ptr<Scorer> Ptr;
  virtual void Setup(int candidate_amount) = 0;
  virtual void Score(std::vector<std::vector<cv::DMatch> > matches) = 0;
  virtual int Result() = 0;
  static double threshold;
};

class NaiveScoreEstimation : public Scorer {
public:
  typedef boost::shared_ptr<NaiveScoreEstimation> Ptr;
  static constexpr int id = 1;
  void Setup(int candidate_amount) override;
  void Score(std::vector<std::vector<cv::DMatch> > matches) override;
  int Result() override;
private:
  std::vector<int> candidates_;
  int most_voted_;
  int most_voted_amount_;
  int total_votes_;
};

/*
class Match {
public:
  struct Scores {
    std::vector<int> candidates;
    int max_voted_candidate;
    int max_votes;
    int total;
  };
  typedef boost::shared_ptr<Match> Ptr;
  Match();
  virtual void TrainDatabase(std::vector<cv::Mat> &database) = 0;
  virtual void MatchDescriptors(std::vector<cv::Mat> &queries, std::vector<cv::Mat> &database,
                                std::vector<std::vector<std::vector<cv::DMatch> > > &matches) = 0;
  virtual void MatchDescriptors(cv::Mat &query, std::vector<std::vector<cv::DMatch> > &matches) = 0;
  void SetCandidates(int amount);
  void ScoreMatchNaively(cv::DMatch match);
  void ResetScores();
  void SaveDatabase();
  struct Scores scores_;
};
*/
} // namespace prototype2

#endif //PROTOTYPE2_SCORE_H
