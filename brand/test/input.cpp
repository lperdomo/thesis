#include "input.h"

template class Input<pcl::PointXYZ>;
template class Input<pcl::PointXYZRGBA>;

template <class T>
Input<T>::Input(float focalX, float focalY, float centerX, float centerY, float scale) :
    cloud(new pcl::PointCloud<T>),
    normalsCloud(new pcl::PointCloud<pcl::Normal>),
    rangeImagePlanar(new pcl::RangeImagePlanar),
    rangeImage(new pcl::RangeImage)
{
    this->focalX = focalX;
    this->focalY = focalY;
    this->centerX = centerX;
    this->centerY = centerY;
    this->scale = scale;
}

template <class T>
void Input<T>::loadImage(std::string file, int type)
{
    mat = cv::Scalar(0,0,0);
    mat = cv::imread(file, type);
}

template <>
void Input<pcl::PointXYZRGBA>::createCloud(cv::Mat depth)
{
    this->createCloudMat(depth);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->clear();
    cloud->width = cloudMat.cols;
    cloud->height = cloudMat.rows;
    cloud->points.resize(cloudMat.cols*cloudMat.rows);
    cloud->is_dense = false;
    for (int y = 0; y < cloudMat.rows; ++y) {
        for (int x = 0; x < cloudMat.cols; ++x) {
            cloud->at(x,y).x = cloudMat.at<cv::Point3f>(y,x).x;
            cloud->at(x,y).y = cloudMat.at<cv::Point3f>(y,x).y;
            cloud->at(x,y).z = cloudMat.at<cv::Point3f>(y,x).z;
            cloud->at(x,y).r = mat.at<cv::Vec3b>(y,x)[2];
            cloud->at(x,y).g = mat.at<cv::Vec3b>(y,x)[1];
            cloud->at(x,y).b = mat.at<cv::Vec3b>(y,x)[0];
        }
    }
    //return depthCloud;
}

template <>
void Input<pcl::PointXYZ>::createCloud()
{
    this->createCloudMat(mat);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->clear();
    cloud->width = cloudMat.cols;
    cloud->height = cloudMat.rows;
    cloud->points.resize(cloudMat.cols*cloudMat.rows);
    cloud->is_dense = false;
    for (int y = 0; y < cloudMat.rows; ++y) {
        for (int x = 0; x < cloudMat.cols; ++x) {
            cloud->at(x,y).x = cloudMat.at<cv::Point3f>(y,x).x;
            cloud->at(x,y).y = cloudMat.at<cv::Point3f>(y,x).y;
            cloud->at(x,y).z = cloudMat.at<cv::Point3f>(y,x).z;
            //if (cloud->at(x,y).y == -1.02027 && cloud->at(x,y).x == 2.43408) {
                //std::cout << "eita" << std::endl;
                //std::cout << "x" << cloud->at(x,y).x << " y" << cloud->at(x,y).y << std::endl;
                //std::cout << "z" << cloud->at(x,y).z << std::endl;
            //}
        }
    }
    //return depthCloud;
}

template <class T>
void Input<T>::createCloudMat(cv::Mat depth)
{
    float inverseFocalX = 1.f/focalX, inverseFocalY = 1.f/focalY;

    cloudMat = cv::Scalar(0,0,0);
    cloudMat.create(depth.size(), CV_32FC3);
    for(int y = 0; y < cloudMat.rows; ++y) {
        cv::Point3f *ptrCloudMat = (cv::Point3f*)cloudMat.ptr(y);
        const uint16_t *ptrDepth = (uint16_t*)depth.ptr(y);
        for(int x = 0; x < cloudMat.cols; ++x) {
            float d = (float)ptrDepth[x]/scale;
            if (d != 0) {
                ptrCloudMat[x].x = (x - centerX) * d * inverseFocalX;
                ptrCloudMat[x].y = (y - centerY) * d * inverseFocalY;
                ptrCloudMat[x].z = d;
            } else {
                ptrCloudMat[x].x = std::numeric_limits<float>::quiet_NaN();
                ptrCloudMat[x].y = std::numeric_limits<float>::quiet_NaN();
                ptrCloudMat[x].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
}

template <class T>
void Input<T>::createRangeImagePlanar()
{
    //pcl::RangeImagePlanar::Ptr rangeImagePlanar(new pcl::RangeImagePlanar);
    std::vector<float> depthData;
    int width = mat.cols, height = mat.rows;
    depthData.resize(width * height);
    float *depthDataBuffer = (float *)&depthData[0];
    for (int i = 0; i < width*height; ++i) {
        depthDataBuffer[i] = cloud->points[i].z;
    }
    rangeImagePlanar->setDepthImage(&depthData[0], mat.cols, mat.rows,
                                    mat.cols/2, mat.rows/2, focalX, focalY);
    rangeImagePlanar->setUnseenToMaxRange();
    //return rangeImagePlanar;
}

template <class T>
void Input<T>::createRangeImage()
{
     Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
     scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                                              cloud->sensor_origin_[1],
                                                              cloud->sensor_origin_[2])) *
                             Eigen::Affine3f(cloud->sensor_orientation_);
     float noise_level = 0.0;
     float min_range = 0.0f;
     int border_size = 1;
     rangeImage->createFromPointCloud(*cloud, pcl::deg2rad(0.5f), pcl::deg2rad(0.5f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                      scene_sensor_pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range, border_size);
     rangeImage->setUnseenToMaxRange();

     /*double rval_tmp;
     IplImage *tmp = cvCreateImage(cvSize(rangeImage->width , rangeImage->height), IPL_DEPTH_32F, 1);
     IplImage *t2 = cvCreateImage(cvSize(rangeImage->width , rangeImage->height), IPL_DEPTH_8U, 1);
     cvSetZero(tmp);
     cvSetZero(t2);

     for(int p_x = 0; p_x < tmp->width; p_x++)
       for(int p_y = 0; p_y < tmp->height; p_y++) {
           rval_tmp = rangeImage->getPoint(p_x, p_y).range;
           if(rval_tmp > 0.0f){
               cvSet2D(tmp, p_y, p_x, cvScalar(rval_tmp));
               if(rval_tmp > 3.0f)
                   t2->imageData[p_y*t2->widthStep + p_x] = 255;
               else if(rval_tmp > 2.0f)
                   t2->imageData[p_y*t2->widthStep + p_x] = 200;
               else if(rval_tmp > 1.0f)
                   t2->imageData[p_y*t2->widthStep + p_x] = 150;
               else if(rval_tmp > 0.5f)
                   t2->imageData[p_y*t2->widthStep + p_x] = 100;
               else
                   t2->imageData[p_y*t2->widthStep + p_x] = 50;
           } else {
               cvSet2D(tmp, p_y, p_x, cvScalar(0.0));
               t2->imageData[p_y*t2->widthStep + p_x] = 0;
           }
       }
     cv::Mat rangeMat(tmp);
     cv::namedWindow("range image");
     cv::imshow("range image", t2);
     cv::waitKey(3);*/
}

template <class T>
void Input<T>::createNormalsCloud()
{
    //pcl::PointCloud<pcl::Normal>::Ptr normalsCloud(new pcl::PointCloud<pcl::Normal>);
    normalsCloud->clear();
    normalsCloud->width = cloud->width;
    normalsCloud->height = cloud->height;
    normalsCloud->points.resize(normalsCloud->width*normalsCloud->height);

    pcl::IntegralImageNormalEstimation<T, pcl::Normal> normalsEstimation;
    normalsEstimation.setInputCloud(cloud);
    normalsEstimation.setNormalSmoothingSize(5);
    normalsEstimation.setNormalEstimationMethod(normalsEstimation.COVARIANCE_MATRIX);
    normalsEstimation.compute(*normalsCloud);
    //std::cout << "Normals estimation" << std::endl;

    normalsMat = cv::Scalar(0,0,0);
    normalsMat.create(mat.size(), CV_32FC3);

    for (size_t y = 0; y < normalsCloud->height; ++y) {
        for (size_t x = 0; x < normalsCloud->width; ++x) {
            normalsMat.at<cv::Point3f>(y,x).x = normalsCloud->at(x,y).normal_x;
            normalsMat.at<cv::Point3f>(y,x).y = normalsCloud->at(x,y).normal_y;
            normalsMat.at<cv::Point3f>(y,x).z = normalsCloud->at(x,y).normal_z;
        }
    }
    //return normalsCloud;
}
