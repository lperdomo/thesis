//
// Created by phi on 16/05/18.
//

#include "output.h"

namespace prototype2 {

cv::Mat Output::output_distance_matrix_;

void Output::ShowQueryResult(pcl::PointCloud<PointType>::ConstPtr query, pcl::PointCloud<PointType>::ConstPtr result) {
  cv::Mat query_image = this->RenderToImage(query);
  cv::Mat output(cv::Size(query_image.cols * 2, query_image.rows * 2), query_image.type(), cv::Scalar::all(255));

  std::string query_text  = "Query frame ";
  std::string result_text = "Result frame ";
  query_text.append(std::to_string(query->header.seq));
  query_image.copyTo(output(cv::Rect(0, query_image.rows, query_image.cols, query_image.rows)));

  if (result) {
    cv::Mat result_image = this->RenderToImage(result);
    result_image.copyTo(output(cv::Rect(query_image.cols, query_image.rows, query_image.cols, query_image.rows)));
    result_text.append(std::to_string(result->header.seq));
  }

  cv::cvtColor(output, output, CV_BGR2RGB); //convert color
  cv::flip(output, output, 0); //align axis with visualizer

  cv::putText(output, "Query", cv::Point(0, query_image.rows), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 1);
  cv::putText(output, "Result", cv::Point(query_image.cols, query_image.rows), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 1);
  cv::putText(output, query_text, cv::Point(0, query_image.rows + 20), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 1);
  cv::putText(output, result_text, cv::Point(0, query_image.rows + 40), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 1);
  cv::imshow("Query result", output);
  cv::waitKey(1);
}

cv::Mat Output::RenderToImage(pcl::PointCloud<PointType>::ConstPtr cloud) {
  std::string name = "cloud";
  name.append(std::to_string(cloud->header.seq));
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> color(cloud);
  pcl::visualization::PCLVisualizer viewer(name, false);
  viewer.setBackgroundColor (255, 255, 255);
  viewer.addPointCloud<PointType>(cloud, color, name);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  viewer.setShowFPS(false);

  pcl::visualization::Camera camera;
  camera.focal[0] = 0;
  camera.focal[1] = 0;
  camera.focal[2] = 0;
  camera.pos[0]   = 0;
  camera.pos[1]   = 0;
  camera.pos[2]   = -2;
  camera.view[0]  = 0;
  camera.view[1]  = -1;
  camera.view[2]  = 0;
  camera.window_size[0] = 320;
  camera.window_size[1] = 240;
  camera.clip[0] = 0.01;
  camera.clip[1] = 10.01;
  camera.fovy = 0.5;
  viewer.setCameraParameters(camera);

  vtkSmartPointer<vtkRenderWindow> renderWindow = viewer.getRenderWindow();
  renderWindow->OffScreenRenderingOn();
  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
  windowToImageFilter->SetInput(renderWindow);
  windowToImageFilter->SetInputBufferTypeToRGB();
  windowToImageFilter->Update();
  vtkImageData* vtkRGBimage = windowToImageFilter->GetOutput();
  int dimension[3];
  vtkRGBimage->GetDimensions(dimension);
  cv::Mat image(dimension[1], dimension[0], CV_8UC3, vtkRGBimage->GetScalarPointer());
  return image;
  //cv::Mat image(dimension[1], dimension[0], CV_8UC3, vtkRGBimage->GetScalarPointer());
  //cv::cvtColor(image, image, CV_BGR2RGB); //convert color
  //cv::flip(image, image, 0); //align axis with visualizer
  //return image;
}

void Output::ShowDistanceMatrix(cv::Mat distance_matrix) {
  cv::imshow("Distance matrix", distance_matrix);
  cv::waitKey(10000);
  //std::cin.get();
}

void Output::Show(cv::Mat& distance_matrix) {
  output_distance_matrix_ = distance_matrix;



}

void Output::BuildOutput() {

}

/*void Output::SaveDistanceMatrix(std::vector<float> matches) {
  std::fstream file;
  file.open(distance_matrix.c_str(), std::ios::out);
  for (size_t i = 0; i < distance_matrix.rows; i++) {
    for (size_t j = 0; j < distance_matrix.cols; j++) {
      file << distance_matrix.at<uchar>(i, j) << ";";
    }
    file << std::endl;
  }
  file.close();
}*/

void Output::distance_matrix(std::string distance_matrix) {
  distance_matrix_ = distance_matrix;
}

} // namespace prototype2
