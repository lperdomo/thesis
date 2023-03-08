#ifndef DETECTOR_H
#define DETECTOR_H

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

namespace pcl {
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ> {
        inline float
        operator () (const PointXYZ &p) const {
            return p.z;
        }
    };
}

class Detector
{
public:
    Detector();

    struct greaterThan
    {
        bool operator()(const cv::KeyPoint& k1, const cv::KeyPoint& k2)
        {
            return k1.pt.x < k2.pt.x && k1.pt.y < k2.pt.y && k1.angle < k2.angle;
        }
    };

    struct equals
    {
        bool operator()(const cv::KeyPoint& k1, const cv::KeyPoint& k2)
        {
            return k1.pt.x == k2.pt.x && k1.pt.y == k2.pt.y && k1.angle == k2.angle;
        }
    };
    void loadCloudIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void combineKeyPoints(std::vector<cv::KeyPoint> right);
    void detectSTARKeyPoints(cv::Mat rgb, cv::Mat depth);
    void detectSTARKeyPoints(cv::Mat rgb);
    void detectSIFTKeyPoints(cv::Mat rgb, cv::Mat depth);
    void detectSIFTKeyPoints(cv::Mat rgb);
    void detectSIFTZKeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void detectNARFKeyPoints(pcl::RangeImagePlanar::Ptr inputRangeImage);
    void detectNARFKeyPoints(pcl::RangeImage::Ptr rangeImage);
    std::vector<std::pair<int, int> > index;
    std::vector<cv::KeyPoint> keyPoints;
    int typeRGB;
    int typeDepth;
};

#endif // DETECTOR_H
