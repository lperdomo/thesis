#ifndef INPUT_H
#define INPUT_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/integral_image_normal.h>

template<class T>
class Input
{
public:
    Input(float focalX, float focalY, float centerX, float centerY, float scale = 5000);
    void loadImage(std::string file, int type);
    void createCloud();
    void createCloud(cv::Mat depth);
    void createRangeImagePlanar();
    void createRangeImage();
    void createNormalsCloud();
    float focalX;
    float focalY;
    float centerX;
    float centerY;
    float scale;
    std::string file;
    cv::Mat mat;
    cv::Mat cloudMat;
    cv::Mat normalsMat;
    cv::Mat rangeImageMat;
    typename pcl::PointCloud<T>::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normalsCloud;
    pcl::RangeImagePlanar::Ptr rangeImagePlanar;
    pcl::RangeImage::Ptr rangeImage;
private:
    void createCloudMat(cv::Mat depth);
};

#endif // INPUT_H
