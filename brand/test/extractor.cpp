#include "extractor.h"

Extractor::Extractor()
{

}

void Extractor::computeBrand(cv::Mat image, cv::Mat cloud, cv::Mat normals, std::vector<cv::KeyPoint> keyPoints)
{
    BrandDescriptorExtractor brand;
    brand.compute(image, cloud, normals, keyPoints, descriptors);
    //std::cout << "Brand descriptor extraction " << descriptors.size() << std::endl;
}

void Extractor::computeSift(cv::Mat image, std::vector<cv::KeyPoint> keyPoints)
{
    cv::SiftDescriptorExtractor extractor;
    extractor.compute(image, keyPoints, descriptors);
    //std::cout << "SIFT descriptor extraction " << descriptors.size() << std::endl;
}
