#include "detector.h"

Detector::Detector()
{

}

void Detector::loadCloudIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (size_t y = 0; y < cloud->height; ++y) {
        for (size_t x = 0; x < cloud->width; ++x) {
            index.push_back(std::make_pair(y,x));
        }
    }
}

void Detector::combineKeyPoints(std::vector<cv::KeyPoint> right)
{
    keyPoints.insert(keyPoints.end(), right.begin(), right.end());
    std::sort(keyPoints.begin(), keyPoints.end(), greaterThan());
    keyPoints.erase(std::unique(keyPoints.begin(), keyPoints.end(), equals()), keyPoints.end());
    //std::cout << "Combined keypoints " << keyPoints.size() << std::endl;
}

void Detector::detectSIFTZKeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointXYZ> detector;
    pcl::PointCloud<pcl::PointXYZ> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());
    detector.setSearchMethod(kdTree);
    detector.setScales(0.005f, 6, 4);//minScale, numOctaves, numScalesPerOctave
    detector.setMinimumContrast(0.005f);//numScalesPerOctave
    detector.setInputCloud(cloud);
    detector.compute(result);
    //std::cout << "SIFT 3D Z key points " << result.points.size () << std::endl;

    std::vector<cv::KeyPoint> keyPoints;
    kdTree->setInputCloud(cloud);
    for (size_t i = 0; i < result.points.size(); i++) {
        int K = 10;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        kdTree->nearestKSearch(result.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
        std::pair<int,int> pi = index.at(pointIdxNKNSearch[0]);
        cv::KeyPoint kp;
        kp.pt.x = pi.second;
        kp.pt.y = pi.first;
        keyPoints.push_back(kp);
    }

    this->combineKeyPoints(keyPoints);
}

void Detector::detectSTARKeyPoints(cv::Mat rgb, cv::Mat depth)
{
    cv::StarFeatureDetector detector;
    std::vector<cv::KeyPoint> keyPointsTemp, keyPoints;
    detector.detect(rgb, keyPointsTemp);
    for (size_t i = 0; i < keyPointsTemp.size(); i++) {
        if (depth.at<uchar>(keyPointsTemp[i].pt.y, keyPointsTemp[i].pt.x) != 0) {
            keyPoints.push_back(keyPointsTemp[i]);
        }
    }
    //std::cout << "STAR 2D RGB key points " << keyPoints.size() << std::endl;
    //this->keyPoints = keyPoints;
    this->combineKeyPoints(keyPoints);
}

void Detector::detectSTARKeyPoints(cv::Mat rgb)
{
    cv::StarFeatureDetector detector;
    std::vector<cv::KeyPoint> keyPoints;
    detector.detect(rgb, keyPoints);
    //std::cout << "STAR 2D RGB key points " << keyPoints.size() << std::endl;
    //this->keyPoints = keyPoints;
    this->combineKeyPoints(keyPoints);
}

void Detector::detectSIFTKeyPoints(cv::Mat rgb, cv::Mat depth)
{
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keyPointsTemp, keyPoints;
    detector.set("nOctaveLayers", 3);
    detector.set("contrastThreshold", 0.04f);
    detector.set("edgeThreshold", 10);
    detector.set("sigma", 1.6f);
    detector.detect(rgb, keyPointsTemp);
    //std::cout << "SIFT 2D RGB key points " << keyPointsTemp.size() << std::endl;
    for (size_t i = 0; i < keyPointsTemp.size(); i++) {
        if (depth.at<uchar>(keyPointsTemp[i].pt.y, keyPointsTemp[i].pt.x) != 0) {
            keyPoints.push_back(keyPointsTemp[i]);
        }
    }
    //std::cout << "SIFT 2D RGB key points " << keyPoints.size() << std::endl;
    //this->keyPoints = keyPoints;
    this->combineKeyPoints(keyPoints);
}

void Detector::detectSIFTKeyPoints(cv::Mat rgb)
{
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keyPoints;
    detector.set("nOctaveLayers", 3);
    detector.set("contrastThreshold", 0.04f);
    detector.set("edgeThreshold", 10);
    detector.set("sigma", 1.6f);
    detector.detect(rgb, keyPoints);
    //std::cout << "SIFT 2D RGB key points " << keyPoints.size() << std::endl;
    //this->keyPoints = keyPoints;
    this->combineKeyPoints(keyPoints);
}

void Detector::detectNARFKeyPoints(pcl::RangeImagePlanar::Ptr rangeImagePlanar)
{
    pcl::RangeImageBorderExtractor rangeImageBorderExtractor;
    pcl::PointCloud<int> result;
    pcl::NarfKeypoint detector(&rangeImageBorderExtractor);
    detector.setRangeImage(&*rangeImagePlanar);
    detector.getParameters().support_size = 0.2f;
    detector.getParameters().min_interest_value = 0.2;
    //detector.getParameters().add_points_on_straight_edges = true;
    detector.getParameters().calculate_sparse_interest_image = false;
    detector.getParameters().use_recursive_scale_reduction = true;
    detector.compute(result);
    //std::cout << "NARF key points " << result.points.size() << std::endl;

    std::vector<cv::KeyPoint> keyPoints;
    for (size_t i = 0; i < result.points.size(); ++i) {
        std::pair<int,int> pi = index.at(result.points[i]);
        cv::KeyPoint kp;
        kp.pt.x = pi.second;
        kp.pt.y = pi.first;
        keyPoints.push_back(kp);
    }
    this->combineKeyPoints(keyPoints);
}

void Detector::detectNARFKeyPoints(pcl::RangeImage::Ptr rangeImage)
{
    pcl::RangeImageBorderExtractor rangeImageBorderExtractor;
    pcl::PointCloud<int> result;
    pcl::NarfKeypoint detector(&rangeImageBorderExtractor);
    detector.setRangeImage(&*rangeImage);
    detector.getParameters().support_size = 0.2f;
    //detector.setRadiusSearch(0.01);
    //detector.getParameters().add_points_on_straight_edges = true;
    detector.getParameters().calculate_sparse_interest_image = false;
    detector.getParameters().use_recursive_scale_reduction = true;
    detector.compute(result);
    //std::cout << "NARF key points " << result.points.size() << std::endl;

    std::vector<cv::KeyPoint> keyPoints;
    for (size_t i = 0; i < result.points.size(); ++i) {
        std::pair<int,int> pi = index.at(result.points[i]);
        cv::KeyPoint kp;
        kp.pt.x = pi.second;
        kp.pt.y = pi.first;
        keyPoints.push_back(kp);
    }
    this->combineKeyPoints(keyPoints);
}
