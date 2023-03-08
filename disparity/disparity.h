#ifndef DISPARITY_H
#define DISPARITY_H

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>

#include "elas.h"
#include "image.h"

namespace image_undistort {

// Default values

// queue size
constexpr int kDepthQueueSize = 10;
// small number used to check things are approximately equal
constexpr double kDelta = 0.000000001;
// stereo parameters
constexpr int kPreFilterCap = 31;//31
constexpr int kSADWindowSize = 5;//11;
constexpr int kMinDisparity = 0;//0
constexpr int kNumDisparities = 64;//64;
constexpr int kUniquenessRatio = 0;//0;
constexpr int kSpeckleRange = 3;//3
constexpr int kSpeckleWindowSize = 250;//500
// bm parameters
constexpr int kTextureThreshold = 0;//0;
const std::string kPreFilterType = "xsobel";
constexpr int kPreFilterSize = 9;//9;
// sgbm parameters
constexpr bool kUseSGBM = false;//false;
constexpr bool kUseELAS = false;//false;
constexpr int kP1 = 0;//120
constexpr int kP2 = 0;//240
constexpr int kDisp12MaxDiff = -1;//-1
constexpr bool kUseHHMode = false;//false

constexpr bool kDoMedianBlur = true;//true;


class Depth {
 public:
  Depth();

  void compute(const std::string frame_n, const cv::Mat& first_image_in, const cv::Mat& second_image_in,
							 double first_camera_info[12], double second_camera_info[12]);
  void compute(const std::string frame_n, const cv::Mat& first_image, const cv::Mat& second_image,
                    double baseline, double focal_length, int cx, int cy);							 
  void calcDisparityImage(const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat& disparity);
  void calcPointCloud(const cv::Mat& input_disparity, const cv::Mat& left_image,
		                  const double baseline, const double focal_length,
					  				  const int cx, const int cy,
				              pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
					            pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud);
  void buildFilledDisparityImage(const cv::Mat& input_disparity,
                                 cv::Mat* disparity_filled,
                                 cv::Mat* input_valid);
  void fillDisparityFromSide(const cv::Mat& input_disparity,
                             const cv::Mat& valid, const bool& from_left,
                             cv::Mat* filled_disparity);
/*
 private:
  static bool ApproxEq(double A, double B);

  bool processCameraInfo(
      const sensor_msgs::CameraInfoConstPtr& first_camera_info,
      const sensor_msgs::CameraInfoConstPtr& second_camera_info,
      double* baseline, double* focal_length, bool* first_is_left, int* cx,
      int* cy);

  static void fillDisparityFromSide(const cv::Mat& input_disparity,
                                    const cv::Mat& valid, const bool& from_left,
                                    cv::Mat* filled_disparity);

  void bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                 cv::Mat* disparity_filled,
                                 cv::Mat* input_valid) const;

  void calcDisparityImage(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                          const sensor_msgs::ImageConstPtr& second_image_msg_in,
                          cv_bridge::CvImagePtr disparity_ptr) const;

  void calcPointCloud(const cv::Mat& input_disparity, const cv::Mat& left_image,
                      const double baseline, const double focal_length,
                      const int cx, const int cy,
                      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
                      pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud);
*/

  // stereo parameters
  int pre_filter_cap_;
  int sad_window_size_;
  int min_disparity_;
  int num_disparities_;
  int uniqueness_ratio_;
  int speckle_range_;
  int speckle_window_size_;

  // bm parameters
  int texture_threshold_;
  int pre_filter_type_;
  int pre_filter_size_;

  // sgbm parameters
  bool use_sgbm_;
  bool use_elas_;
  int p1_;
  int p2_;
  int disp_12_max_diff_;
  bool use_mode_HH_;

  bool do_median_blur_;

};

}

#endif
