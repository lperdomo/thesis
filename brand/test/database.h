#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

class Database
{
public:
    Database();
    void loadTest1();
    void loadTest2();
    void loadTest2Queries();
    void loadTest3();
    void load();
    void loadQueries();
    std::vector<std::string> filesRGB;
    std::vector<std::string> filesDepth;
    std::vector<std::vector<cv::KeyPoint> > keyPoints;
    std::vector<cv::Mat> descriptors;
    std::vector<int> queryTarget;
};

#endif // DATABASE_H
