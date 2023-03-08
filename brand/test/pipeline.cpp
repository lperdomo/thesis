#include "pipeline.h"

const int Pipeline::NONE = 0;
const int Pipeline::BRAND = 1;
const int Pipeline::NARF = 2;
const int Pipeline::SIFT = 3;
const int Pipeline::STAR = 4;

Pipeline::Pipeline() :
    depth(525.0f, 525.0f, 319.5f, 239.5f, 5000),
    rgb(517.3f, 516.5f, 318.6f, 255.3f), //fr1
    detector(), extractor()
    //rgb(520.9f, 521.0f, 325.1f, 249.7f); //fr2
{

}

void Pipeline::run(std::string rgbFile, std::string depthFile)
{
    depth.loadImage(depthFile, CV_LOAD_IMAGE_ANYDEPTH);
    rgb.loadImage(rgbFile, CV_LOAD_IMAGE_COLOR);
    depth.createCloud();
    rgb.createCloud(depth.mat);

    detector.keyPoints.clear();
    detector.loadCloudIndices(depth.cloud);

    //KEYPOINTS
    if (detector.typeRGB == STAR) {
        if (extractor.type != BRAND) {
            detector.detectSTARKeyPoints(rgb.mat);
        } else {
            detector.detectSTARKeyPoints(rgb.mat, depth.mat);
        }
    } else if (detector.typeRGB == SIFT) {
        if (extractor.type != BRAND) {
            detector.detectSIFTKeyPoints(rgb.mat);
        } else {
            detector.detectSIFTKeyPoints(rgb.mat, depth.mat);
        }
    }
    if (detector.typeDepth == NARF) {
        depth.createRangeImagePlanar();
        detector.detectNARFKeyPoints(depth.rangeImagePlanar);
    } else if (detector.typeDepth == SIFT) {
        detector.detectSIFTZKeyPoints(depth.cloud);
    }

    //DESCRIPTORS
    if (extractor.type == SIFT) {
        extractor.computeSift(rgb.mat, detector.keyPoints);
    } else if (extractor.type == BRAND) {
        //CALCULATE NORMALS
        depth.createNormalsCloud();
        extractor.computeBrand(rgb.mat, depth.mat, depth.normalsMat, detector.keyPoints);
    }

}
