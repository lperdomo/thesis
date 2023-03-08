#include <iostream>

#include "disparity.h"

int main(int argc, char **argv) {
  image_undistort::Depth depth;
  std::string line, line_2;
  double first_camera_info[12], second_camera_info[12];
  double baseline, focal_length;
  int cx, cy;
  std::string dataset = "malaga";
  
  if (dataset == "kitti") {
	std::string label;
	std::ifstream file("/home/phi/CLionProjects/kitti/00/calib.txt");
	std::getline(file, line);//P0, left grayscale
	std::getline(file, line);//P1, right grayscale
	std::getline(file, line);//P2, left color
	std::stringstream ssl(line);
	ssl >> label
	    >> first_camera_info[0]  >> first_camera_info[1]
		>> first_camera_info[2]  >> first_camera_info[3]
		>> first_camera_info[4]  >> first_camera_info[5]
		>> first_camera_info[6]  >> first_camera_info[7]
		>> first_camera_info[8]  >> first_camera_info[9]
	    >> first_camera_info[10] >> first_camera_info[11];
	std::getline(file, line);//P3, right color
	std::stringstream ssr(line);
	ssr >> label
	    >> second_camera_info[0]  >> second_camera_info[1]
		>> second_camera_info[2]  >> second_camera_info[3]
		>> second_camera_info[4]  >> second_camera_info[5]
		>> second_camera_info[6]  >> second_camera_info[7]
		>> second_camera_info[8]  >> second_camera_info[9]
		>> second_camera_info[10] >> second_camera_info[11];
	file.close();
	cv::Mat left_image, right_image;

	for (int i = 0; i < 4541; i++) {
      std::string left_path = "/home/phi/CLionProjects/kitti/00/image_2/";
      std::string right_path = "/home/phi/CLionProjects/kitti/00/image_3/";
	  std::string frame_n = std::string((6 - (int) ((i < 10) ? 1 : log10(i) + 1)), '0').append(std::to_string(i));
	  left_path.append(frame_n);
	  left_path.append(".png");
	  right_path.append(frame_n);
	  right_path.append(".png");
	  left_image = cv::imread(left_path, CV_32F);
	  right_image = cv::imread(right_path, CV_32F);
	  depth.compute(frame_n, left_image, right_image, first_camera_info, second_camera_info);
	}
  } else if (dataset == "santalucia") {
	baseline = 0.75066669;
	focal_length = 1246.5617;
	cx = 532.2879;
	cy = 383.6041;
	cv::Mat left_image, right_image;
	double timestamp;
	for (int i = 0; i < 3287; i++) {
	  std::string left_path = "/home/phi/CLionProjects/santalucia/left/";
  	  std::string right_path = "/home/phi/CLionProjects/santalucia/right/";
	  std::string frame_n = std::to_string(i);
	  left_path.append(frame_n);
	  left_path.append(".png");
	  right_path.append(frame_n);
	  right_path.append(".png");
	  left_image = cv::imread(left_path, CV_32F);
	  right_image = cv::imread(right_path, CV_32F);
	  depth.compute(frame_n, left_image, right_image, baseline, focal_length, cx, cy);
	}
  } else if (dataset == "malaga") {	  
	baseline = 0.119471;
	focal_length = 621.18428;
	cx = 404.00760;
	cy = 309.05989;
	cv::Mat left_image, right_image;
	double timestamp;
	for (int i = 0; i < 502; i++) {
	  std::string left_path = "/home/phi/CLionProjects/malaga/08/left/";
  	  std::string right_path = "/home/phi/CLionProjects/malaga/08/right/";
	  std::string frame_n = std::to_string(i);
	  left_path.append(frame_n);
	  left_path.append(".jpg");
	  right_path.append(frame_n);
	  right_path.append(".jpg");
	  left_image = cv::imread(left_path, CV_32F);
	  right_image = cv::imread(right_path, CV_32F);
	  depth.compute(frame_n, left_image, right_image, baseline, focal_length, cx, cy);
	}
  }

  std::cout << "eeeee" <<std::endl;
  return 1;
}
