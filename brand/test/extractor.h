#ifndef EXTRACTOR_H
#define EXTRACTOR_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/range_image_border_extractor.h>

#include "brand.h"

class Extractor
{
public:
    Extractor();
    void computeBrand(cv::Mat image, cv::Mat cloud, cv::Mat normals, std::vector<cv::KeyPoint> keyPoints);
    void computeSift(cv::Mat image, std::vector<cv::KeyPoint> keyPoint);
    cv::Mat descriptors;
    int type;
};

#endif // EXTRACTOR_H
