#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "detector.h"
#include "match.h"
#include "input.h"
#include "extractor.h"
#include "pipeline.h"
#include "database.h"

int main(int argc, char *argv[])
{
    Database database, query;
    //database.loadTest2();
    //query.loadTest2Queries();
    database.load();
    query.loadQueries();
    //query.loadTest3();

    Pipeline pipeline;
    pipeline.detector.typeRGB = Pipeline::SIFT;
    //pipeline.detector.typeDepth = Pipeline::SIFT;
    pipeline.extractor.type = Pipeline::SIFT;
    //pipeline.detector.typeRGB = Pipeline::STAR;
    //pipeline.detector.typeRGB = Pipeline::STAR;
    pipeline.detector.typeDepth = Pipeline::NONE;
    //pipeline.extractor.type = Pipeline::BRAND;
    Match matcher(cv::NORM_L2); //SIFT
    //Match matcher(cv::NORM_HAMMING); //BRAND

    //LOAD DATABASE
    for (size_t i = 0; i < database.filesDepth.size(); ++i) {
        pipeline.run(database.filesRGB.at(i), database.filesDepth.at(i));
        database.descriptors.push_back(pipeline.extractor.descriptors);
        database.keyPoints.push_back(pipeline.detector.keyPoints);
    }

    std::cout << "database" << database.descriptors.size() << std::endl;

    //MATCHING
    for (size_t i = 0; i < query.filesDepth.size(); ++i) {
        std::cout << "MATCH " << i << std::endl;
        pipeline.run(query.filesRGB.at(i), query.filesDepth.at(i));
        int betterCandidate = -1, maxMates = 0;
        for (size_t j = 0; j < database.filesDepth.size(); ++j) {
            matcher.matchBrandDescriptors(pipeline.extractor.descriptors, database.descriptors.at(j));
            matcher.filterMatesWithRANSAC(pipeline.detector.keyPoints, database.keyPoints.at(j));
            std::cout << j << " - " << matcher.mates.size() << std::endl;
            if (matcher.mates.size() > maxMates) {
                maxMates = matcher.mates.size();
                betterCandidate = j;
            }
        }
        std::cout << "Found " << database.filesRGB.at(betterCandidate) << " (" << betterCandidate << ") "
                  << " with " << maxMates << ". Expected " << database.filesRGB.at(query.queryTarget.at(i))
                  << " (" << query.queryTarget.at(i) << ") " << std::endl;

    }

    /*cv::Mat out, imgL, imgR;
    imgL = cv::imread(database.filesRGB.at(0), CV_LOAD_IMAGE_COLOR);
    imgR = cv::imread(query.filesRGB.at(0), CV_LOAD_IMAGE_COLOR);
    matcher.matchBrandDescriptors(pipeline.extractor.descriptors, database.descriptors.at(0));
    matcher.filterMatesWithRANSAC(pipeline.detector.keyPoints, database.keyPoints.at(0), 50, 0.99);

    std::vector<char> mask(matcher.mates.size(), 1);
    std::fill(mask.begin(), mask.begin() + 100, 1);

    std::sort(matcher.mates.begin(), matcher.mates.end(), [](const cv::DMatch& lhs, const cv::DMatch& rhs){ return lhs.distance < rhs.distance; });

    cv::drawMatches(imgL, pipeline.detector.keyPoints, imgR, database.keyPoints.at(0), matcher.mates, out, cv::Scalar::all(-1), cv::Scalar::all(-1), mask);
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window", out);
    cv::waitKey(0);*/


    //std::cout << "img0 eq img0" << std::endl;
    //out = cv::imread(depthFiles.at(0), CV_LOAD_IMAGE_COLOR);
    //cv::drawKeypoints(out, databaseKP.at(0), out);
    //cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    //cv::imshow("Display window", out);
    //cv::waitKey(0);


    //matcher.matchSiftDescriptors(databaseDescriptors.at(0), databaseDescriptors.at(1));
/*
    imgL = cv::imread(databaseRGBFiles.at(0), CV_LOAD_IMAGE_COLOR);
    imgR = cv::imread(databaseRGBFiles.at(3), CV_LOAD_IMAGE_COLOR);
    cv::drawMatches(imgL, databaseKeyPoints.at(0), imgR, databaseKeyPoints.at(3), matcher.mates, out);
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window", out);
    cv::waitKey(0);
*/

   //OUTPUT
    //auto v = std::make_shared<pcl::visualization::CloudViewer>("Viewer");
    //v->showCloud(rgb.cloud);

    //while (!v->wasStopped()) {
    //}
}
