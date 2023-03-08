#include "match.h"

Match::Match(int distanceType)
{
    this->distanceType = distanceType;
}

void Match::matchBrandDescriptors(cv::Mat left, cv::Mat right)
{
    cv::BFMatcher matcher(distanceType);
    mates.clear();
    matcher.knnMatch(left, right, matesLR, 1);
    matcher.knnMatch(right, left, matesRL, 1);
    for (size_t m = 0; m < matesLR.size(); m++ ) {
        bool found = false;
        for (size_t fk = 0; fk < matesLR[m].size(); fk++) {
            cv::DMatch forward = matesLR[m][fk];
            for (size_t bk = 0; bk < matesRL[forward.trainIdx].size(); bk++) {
                cv::DMatch backward = matesRL[forward.trainIdx][bk];
                if (backward.trainIdx == forward.queryIdx) {
                    mates.push_back(forward);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }
}

void Match::matchSiftDescriptors(cv::Mat left, cv::Mat right)
{
    cv::BFMatcher matcher(distanceType);
    mates.clear();
    matcher.knnMatch(left, right, matesLR, 2);
    matcher.knnMatch(right, left, matesRL, 2);
    matesLR = this->filterMatesWithDistanceRatioThreshold(matesLR);
    matesRL = this->filterMatesWithDistanceRatioThreshold(matesRL);
    //std::cout << mates.size() <<" Inliers" << std::endl;
    this->filterMatesWithSymmetry();
    //std::cout << mates.size() <<" Inliers after SYM" << std::endl;
    /*for (size_t m = 0; m < matesLR.size(); m++ ) {
        bool found = false;
        for (size_t fk = 0; fk < matesLR[m].size(); fk++) {
            cv::DMatch forward = matesLR[m][fk];
            for (size_t bk = 0; bk < matesRL[forward.trainIdx].size(); bk++) {
                cv::DMatch backward = matesRL[forward.trainIdx][bk];
                if (backward.trainIdx == forward.queryIdx) {
                    mates.push_back(forward);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }*/
}

std::vector<cv::DMatch> Match::filterMatesWithMaxDistance()
{
    double maxDistance = 0;
    std::vector<cv::DMatch> goodMates;
    for (size_t i = 0; i < mates.size(); i++) {
        double distance = mates[i].distance;
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    double threshold = maxDistance*0.5;
    for (size_t i = 0; i < mates.size(); i++) {
        if (mates[i].distance <= threshold) {
            goodMates.push_back(mates[i]);
        }
    }

    if (goodMates.size() > 3) {
        std::cout << goodMates.size() <<" \"Good mates\"" << std::endl;
    } else {
        std::cout << "Too few good mates" << std::endl;
        goodMates = mates;
    }
    return goodMates;
}

std::vector<std::vector<cv::DMatch> > Match::filterMatesWithDistanceRatioThreshold(std::vector<std::vector<cv::DMatch> > input, float ratio)
{
    std::vector<std::vector<cv::DMatch> > goodMates;
    for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator = input.begin(); matchIterator != input.end(); ++matchIterator) {
        if (matchIterator->size() > 1) { // have at least 2 neighbours
            if ((*matchIterator)[0].distance/(*matchIterator)[1].distance <= ratio) {
                goodMates.push_back(*matchIterator);
            }
        }
    }
    return goodMates;
}

void Match::filterMatesWithSymmetry()
{
    for (std::vector<std::vector<cv::DMatch> >::iterator matchIteratorL = matesLR.begin(); matchIteratorL != matesLR.end(); ++matchIteratorL) {
        if (matchIteratorL->size() >= 2) {
            for (std::vector<std::vector<cv::DMatch> >::iterator matchIteratorR = matesRL.begin(); matchIteratorR != matesRL.end(); ++matchIteratorR) {
                if (matchIteratorR->size() >= 2) {
                    if ((*matchIteratorL)[0].queryIdx == (*matchIteratorR)[0].trainIdx  &&
                        (*matchIteratorR)[0].queryIdx == (*matchIteratorL)[0].trainIdx) {
                            mates.push_back(cv::DMatch((*matchIteratorL)[0].queryIdx,
                                                       (*matchIteratorL)[0].trainIdx,
                                                       (*matchIteratorL)[0].distance));
                            break;
                    }
                }
            }
        }
    }
}

void Match::filterMatesWithRANSAC(std::vector<cv::KeyPoint> left, std::vector<cv::KeyPoint> right, double epipolarDistance, double confidence)
{
    std::vector<cv::DMatch> goodMates;

    std::vector<cv::Point2f> pointsL, pointsR;
    for (std::vector<cv::DMatch>::const_iterator it = mates.begin(); it!= mates.end(); ++it) {
        pointsL.push_back(cv::Point2f(left[it->queryIdx].pt.x, left[it->queryIdx].pt.y));
        pointsR.push_back(cv::Point2f(right[it->queryIdx].pt.x, right[it->queryIdx].pt.y));
    }

    std::vector<uchar> inliers(pointsL.size(), 0);
    cv::Mat fundamental = cv::findFundamentalMat(cv::Mat(pointsL), cv::Mat(pointsR),
                                                 inliers, CV_FM_RANSAC,
                                                 epipolarDistance, confidence);
    std::vector<uchar>::iterator itInliers = inliers.begin();
    std::vector<cv::DMatch>::iterator itMates = mates.begin();

    for ( ; itInliers!= inliers.end(); ++itInliers, ++itMates) {
        if (*itInliers) {
            goodMates.push_back(*itMates);
        }
    }

    //mates.clear();
    mates = goodMates;
    //std::cout << "After RANSAC" << mates.size() << std::endl;
}
