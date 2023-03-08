//
// Created by phi on 16/05/18.
//

#ifndef PROTOTYPE2_OUTPUT_H
#define PROTOTYPE2_OUTPUT_H

#include "util.h"

#include <GL/glut.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

namespace prototype2 {

class Output {
public:
  typedef boost::shared_ptr<Output> Ptr;
  void ShowQueryResult(typename pcl::PointCloud<PointType>::ConstPtr query, typename pcl::PointCloud<PointType>::ConstPtr result);
  void ShowDistanceMatrix(cv::Mat distance_matrix);
  void Show(cv::Mat& distance_matrix);
  static void BuildOutput();
  static GLuint MatToTexture(cv::Mat &image, GLenum min_filter, GLenum mag_filter, GLenum wrap_filter);
  //void SaveMatches(std::vector<std::vector<float> > matches);
  cv::Mat RenderToImage(typename pcl::PointCloud<PointType>::ConstPtr cloud);
  void distance_matrix(std::string distance_matrix);
private:
  std::string distance_matrix_;
  static cv::Mat output_distance_matrix_;
};

} // namespace prototype2

#endif //PROTOTYPE2_OUTPUT_H
