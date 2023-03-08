#include "disparity.h"

namespace image_undistort {

Depth::Depth() {

  if (kPreFilterType == std::string("xsobel")) {
    pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
  } else if (kPreFilterType == std::string("normalized_response")) {
    pre_filter_type_ = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
  }

  // general stereo parameters
  min_disparity_ = kMinDisparity;
  num_disparities_ = kNumDisparities;
  pre_filter_cap_ = kPreFilterCap;
  uniqueness_ratio_ = kUniquenessRatio;
  speckle_range_ = kSpeckleRange;
  speckle_window_size_ = kSpeckleWindowSize;
  sad_window_size_ = kSADWindowSize;

  // bm parameters
  texture_threshold_ = kTextureThreshold;
  pre_filter_size_ = kPreFilterSize;

  // sgbm parameters
  use_sgbm_ = kUseSGBM;
  use_elas_ = kUseELAS;
  p1_ = kP1;
  p2_ = kP2;
  disp_12_max_diff_ = kDisp12MaxDiff;
  use_mode_HH_ = kUseHHMode;

  do_median_blur_ = kDoMedianBlur;
}

void Depth::calcDisparityImage(const cv::Mat& left_image, const cv::Mat& right_image,
							   cv::Mat& disparity) {
  cv::Mat left_gray, right_gray;
  cv::cvtColor(left_image, left_gray, CV_BGR2GRAY);
  cv::cvtColor(right_image, right_gray, CV_BGR2GRAY);
  if (use_sgbm_) {
    cv::StereoSGBM left_matcher;
    left_matcher.numberOfDisparities = num_disparities_;
    left_matcher.SADWindowSize = sad_window_size_;
    left_matcher.preFilterCap = pre_filter_cap_;
    left_matcher.minDisparity = min_disparity_;
    left_matcher.uniquenessRatio = uniqueness_ratio_;
    left_matcher.speckleRange = speckle_range_;
    left_matcher.speckleWindowSize = speckle_window_size_;
    left_matcher.P1 = p1_;
    left_matcher.P2 = p2_;
    left_matcher.disp12MaxDiff = disp_12_max_diff_;
    left_matcher.fullDP = use_mode_HH_;
    left_matcher.operator()(left_gray, right_gray, disparity);
  } else if (use_elas_) {
	// load images
	image<uchar> *I1,*I2;
	std::cout << "left width " << left_image.cols << " left height " << left_image.rows << std::endl;
	std::cout << "right width " << left_image.cols << " right height " << left_image.rows << std::endl;
	I1 = new image<uchar>(left_image.cols, left_image.rows, left_image.data);
	I2 = new image<uchar>(right_image.cols, right_image.rows, right_image.data);

	// get image width and height
	int32_t width  = I1->width();
	int32_t height = I1->height();

	// allocate memory for disparity images
	const int32_t dims[3] = {width,height,width}; // bytes per line = width
	float* D1_data = (float*)malloc(width*height*sizeof(float));
	float* D2_data = (float*)malloc(width*height*sizeof(float));
	std::cout << "teste 1" << std::endl;
	// process
	Elas::parameters param;
	param.postprocess_only_left = false;
	Elas elas(param);
	elas.process(I1->data,I2->data,D1_data,D2_data,dims);
	std::cout << "teste 2" << std::endl;
	// find maximum disparity for scaling output disparity images to [0..255]
	float disp_max = 0;
	for (int32_t i=0; i<width*height; i++) {
	  if (D1_data[i]>disp_max) disp_max = D1_data[i];
	  if (D2_data[i]>disp_max) disp_max = D2_data[i];
	}
	std::cout << "teste 3" << std::endl;
	// copy float to uchar
	image<uchar> *D1 = new image<uchar>(width,height);
	image<uchar> *D2 = new image<uchar>(width,height);
	//disparity = cv::Mat(height, width, CV_8UC1);
	//std::cout << "width " << width << " height " << height << std::endl;
	//std::cout << "disparity width " << disparity.cols << " height " << disparity.rows << std::endl;
	//for (int32_t i=0; i<height; i++) {
	//  for (int32_t j=0; j<width; j++) {
	for (int32_t i=0; i<width*height; i++) {
	    D1->data[i] = (uint8_t)std::max(255.0*D1_data[i]/disp_max,0.0);
	    D2->data[i] = (uint8_t)std::max(255.0*D2_data[i]/disp_max,0.0);
	    //disparity[count] = (uint8_t)std::max(255.0*D1_data[i]/disp_max,0.0);
	    //disparity.at<uchar>(i, j) = (uint8_t)std::max(255.0*D1_data[i*j]/disp_max,0.0);
	  //}
	}
	std::cout << "teste 4" << std::endl;	  
	cv::Mat teste = cv::Mat(height, width, CV_8UC1, *D1->data);
	disparity = teste.clone();
	std::cout << "teste 5" << std::endl;	  
		  
  } else {
    cv::StereoBM left_matcher;
    left_matcher.state->numberOfDisparities = 128;
    left_matcher.state->SADWindowSize = 5;
    left_matcher.state->preFilterCap = 31;
    left_matcher.state->preFilterSize = 9;
    left_matcher.state->minDisparity = 0;
    left_matcher.state->textureThreshold = 0;
    left_matcher.state->uniquenessRatio = 0;
    left_matcher.state->speckleRange = 3;
    left_matcher.state->speckleWindowSize = 250;
    left_matcher.state->disp12MaxDiff = -1;
    left_matcher.operator()(left_gray, right_gray, disparity);
    cv::normalize(disparity, disparity, 0, 255, CV_MINMAX);

    // we convert the values from 16 bits signed to 8 bits unsigned
    cv::Mat disp(disparity.rows, disparity.cols, CV_8UC1);
    for (int i=0; i<disparity.rows; i++)
        for (int j=0; j<disparity.cols; j++)
            disp.at<unsigned char>(i,j) = (unsigned char)disparity.at<short>(i,j);

    // we convert from gray to color
    cv::Mat disp_color;
    cv::cvtColor(disp, disp_color, CV_GRAY2RGB);
    cv::imshow("pepepepe", disp_color);
    cv::waitKey(0);
  }
  if (do_median_blur_) {
    cv::medianBlur(disparity, disparity, 5);
  }  
  
}

// simply replaces invalid disparity values with a valid value found by scanning
// horizontally (note: if disparity values are already valid or if no valid
// value can be found int_max is inserted)
void Depth::fillDisparityFromSide(const cv::Mat& input_disparity,
                                  const cv::Mat& valid, const bool& from_left,
                                  cv::Mat* filled_disparity) {
  *filled_disparity =
  cv::Mat(input_disparity.rows, input_disparity.cols, CV_16S);

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    bool prev_valid = false;
    int16_t prev_value;

    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      size_t x_scan;
      if (from_left) {
        x_scan = x_pixels;
      } else {
        x_scan = (input_disparity.cols - x_pixels - 1);
      }

      if (valid.at<uint8_t>(y_pixels, x_scan)) {
        prev_valid = true;
        prev_value = input_disparity.at<int16_t>(y_pixels, x_scan);
        filled_disparity->at<int16_t>(y_pixels, x_scan) =
        std::numeric_limits<int16_t>::max();
      } else if (prev_valid) {
        filled_disparity->at<int16_t>(y_pixels, x_scan) = prev_value;
      } else {
        filled_disparity->at<int16_t>(y_pixels, x_scan) =
        std::numeric_limits<int16_t>::max();
      }
    }
  }
}

void Depth::buildFilledDisparityImage(const cv::Mat& input_disparity,
                                      cv::Mat* disparity_filled,
                                      cv::Mat* input_valid) {
  // mark valid pixels
  *input_valid = cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

  int side_bound = sad_window_size_ / 2;

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      // the last check is because the sky has a bad habit of having a disparity
      // at just less than the max disparity
      if ((x_pixels < side_bound + min_disparity_ + num_disparities_) ||
          (y_pixels < side_bound) ||
          (x_pixels > (input_disparity.cols - side_bound)) ||
          (y_pixels > (input_disparity.rows - side_bound)) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) < 0) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) >=
           (min_disparity_ + num_disparities_ - 1) * 16)) {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 0;
      } else {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 1;
      }
    }
  }

  // erode by size of SAD window, this prevents issues with background pixels
  // being given the same depth as neighboring objects in the foreground.
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(sad_window_size_, sad_window_size_));
  cv::erode(*input_valid, *input_valid, kernel);

  // take a guess for the depth of the invalid pixels by scanning along the row
  // and giving them the same value as the closest horizontal point.
  cv::Mat disparity_filled_left, disparity_filled_right;
  fillDisparityFromSide(input_disparity, *input_valid, true,
  						&disparity_filled_left);
  fillDisparityFromSide(input_disparity, *input_valid, false,
  						&disparity_filled_right);

  // take the most conservative disparity of the two
  *disparity_filled = cv::max(disparity_filled_left, disparity_filled_right);

  // 0 disparity is valid but cannot have a depth associated with it, because of
  // this we take these points and replace them with a disparity of 1.
  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      if (input_disparity.at<int16_t>(y_pixels, x_pixels) == 0) {
        disparity_filled->at<int16_t>(y_pixels, x_pixels) = 1;
      }
    }
  }
}

void Depth::calcPointCloud(const cv::Mat& input_disparity, const cv::Mat& left_image,
						   const double baseline, const double focal_length, 
						   const int cx, const int cy, 
						   pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
						   pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud) {
  pointcloud->clear();
  freespace_pointcloud->clear();
  cv::Mat disparity_filled, input_valid;
  
  buildFilledDisparityImage(input_disparity, &disparity_filled, &input_valid);

  int side_bound = sad_window_size_ / 2;

  // build pointcloud
  for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
       ++y_pixels) {
    for (int x_pixels = side_bound + min_disparity_ + num_disparities_;
         x_pixels < input_disparity.cols - side_bound; ++x_pixels) {
      const uint8_t& is_valid = input_valid.at<uint8_t>(y_pixels, x_pixels);
      const int16_t& input_value =
              input_disparity.at<int16_t>(y_pixels, x_pixels);
      const int16_t& filled_value =
              disparity_filled.at<int16_t>(y_pixels, x_pixels);

      bool freespace;
      double disparity_value;

      // if the filled disparity is valid it must be a freespace ray
      if (filled_value < std::numeric_limits<int16_t>::max()) {
        disparity_value = static_cast<double>(filled_value);
        freespace = true;
      }
        // else it is a normal ray
      else if (is_valid) {
        disparity_value = static_cast<double>(input_value);
        freespace = false;
      } else {
        continue;
      }

      pcl::PointXYZRGB point;

      // the 16* is needed as opencv stores disparity maps as 16 * the true
      // values
      point.z = (16 * focal_length * baseline) / disparity_value;
      point.x = point.z * (x_pixels - cx) / focal_length;
      point.y = point.z * (y_pixels - cy) / focal_length;

      if (left_image.channels() == 3) {
        const cv::Vec3b& color = left_image.at<cv::Vec3b>(y_pixels, x_pixels);
        //std::cout << color[0] << ", " << color[1] << ", " << color[2] << std::endl;
        point.b = color[0];
        //point.b = 0;
        point.g = color[1];
        //point.g = 0;
        point.r = color[2];
        //point.r = 255;
      } else if (left_image.channels() == 4) {
        const cv::Vec4b& color = left_image.at<cv::Vec4b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else {
        point.b = left_image.at<uint8_t>(y_pixels, x_pixels);
        point.g = point.b;
        point.r = point.b;
      }

      if (freespace) {
        freespace_pointcloud->push_back(point);
      } else {
        pointcloud->push_back(point);
      }
    }
  }
}

void Depth::compute(const std::string frame_n, const cv::Mat& first_image, const cv::Mat& second_image,
                    double baseline, double focal_length, int cx, int cy) {
  
  bool first_is_left;
	
  if (baseline > 0) first_is_left = false;
  else {
    first_is_left = true;
    baseline *= -1;
  }

  cv::Mat left_image;
  cv::Mat right_image;
  cv::Mat disparity;

  if (first_is_left) {
    left_image = first_image;
    right_image = second_image;
  } else {
    left_image = second_image;
    right_image = first_image;
  }
  std::string file_pc(frame_n+".pcd"),
              file_fpc("f_"+frame_n+".pcd"),
              file_d("d_"+frame_n+".png");


  cv::Mat disparity_image = cv::Mat(left_image.rows, left_image.cols, CV_16S);

  calcDisparityImage(left_image, right_image, disparity);
  cv::imwrite(file_d, disparity);

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> freespace_pointcloud;
  
  calcPointCloud(disparity, left_image, baseline, focal_length,
                 cx, cy, &pointcloud, &freespace_pointcloud);

  pcl::io::savePCDFileASCII(file_pc, pointcloud);
  //pcl::io::savePCDFileASCII(file_fpc, freespace_pointcloud);
}

void Depth::compute(const std::string frame_n, const cv::Mat& first_image, const cv::Mat& second_image,
                    double first_camera_info[12], double second_camera_info[12]) {
  double baseline, focal_length;
  bool first_is_left;
  int cx, cy;

  focal_length = first_camera_info[0];
  baseline = (second_camera_info[3] - first_camera_info[3]) / first_camera_info[0];
  if (baseline > 0) first_is_left = false;
  else {
    first_is_left = true;
    baseline *= -1;
  }
  cx = first_camera_info[2];
  cy = first_camera_info[6];

  cv::Mat left_image;
  cv::Mat right_image;
  cv::Mat disparity;

  if (first_is_left) {
    left_image = first_image;
    right_image = second_image;
  } else {
    left_image = second_image;
    right_image = first_image;
  }

  cv::Mat disparity_image = cv::Mat(left_image.rows, left_image.cols, CV_16S);

  calcDisparityImage(left_image, right_image, disparity);

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> freespace_pointcloud;
  
  calcPointCloud(disparity, left_image, baseline, focal_length,
                 cx, cy, &pointcloud, &freespace_pointcloud);

  std::string file_pc(frame_n+".pcd"),
              file_fpc("f_"+frame_n+".pcd"),
              file_d("d_"+frame_n+".png");
  pcl::io::savePCDFileASCII(file_pc, pointcloud);
  //pcl::io::savePCDFileASCII(file_fpc, freespace_pointcloud);
  cv::imwrite(file_d, disparity);
}

}
