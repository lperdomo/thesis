#ifndef MATCH_H
#define MATCH_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class Match
{
public:
    Match(int distanceType);
    void matchBrandDescriptors(cv::Mat left, cv::Mat right);
    void matchSiftDescriptors(cv::Mat left, cv::Mat right);
    std::vector<cv::DMatch> filterMatesWithMaxDistance();
    std::vector<std::vector<cv::DMatch> > filterMatesWithDistanceRatioThreshold(std::vector<std::vector<cv::DMatch> > input, float ratio = 0.65f);
    void filterMatesWithSymmetry();
    void filterMatesWithRANSAC(std::vector<cv::KeyPoint> left, std::vector<cv::KeyPoint> right, double epipolarDistance = 300, double confidence = 0.99);
    int distanceType;
    std::vector<cv::DMatch> mates;
    std::vector<std::vector<cv::DMatch> > matesLR;
    std::vector<std::vector<cv::DMatch> > matesRL;

};

#endif // MATCH_H
