//
// Created by leo on 01/05/18.
//

#include <QApplication>

#include <iostream>

#include "util.h"

#include "config.h"
#include "pipeline.h"
#include "distances_widget.h"

int main(int argc, char **argv) {

  /*std::vector<float> teste(5);
  teste[0] = 0.5f;
  teste[1] = 0.2f;
  teste[2] = 0.4f;
  teste[3] = 0.1f;
  teste[4] = 0.3f;

  int K = 2;
  std::vector<float> result(K, std::numeric_limits<float>::max());

  for (int i = 0; i < teste.size(); i++) {
      if (teste[i] < result[K-1]) {
        int k = K-2;
        std::cout << "k=" << k << std::endl;
        std::cout << "result[k] > teste[i] = " << result[k] << " > " << teste[i] << std::endl;
        for (k = K-2; k >= 0 && result[k] > teste[i]; k--) {
          result[k+1] = result[k];
        }
        result[k+1] = teste[i];
        for (int j = 0; j < K; j++) std::cout << result[j] << std::endl;
        std::cout << "-----" << std::endl;
      }
  }

  for (int i = 0; i < result.size(); i++) {
    std::cout << result[i] << std::endl;
  }
  std::exit(0);*/



  //QApplication app(argc, argv);
  prototype2::Pipeline pipeline(argc, argv);
  prototype2::Config file(argc, argv);
  int code;
  file.Open();
  file.Load(pipeline);
  file.Close();
  pipeline.Run();

  //code = app.exec();
  //return code;

  /*prototype2::Config file;
  file.Open(std::string(argv[1]));
  prototype2::Dataset::Ptr dataset = file.LoadDataset();
  file.Close();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = dataset->LoadCloud<pcl::PointXYZRGB, prototype2::PointType>(0);
  cv::Mat descriptor;
  prototype2::M2dpEstimation<prototype2::M2dp> m2dp;
  m2dp.input_cloud(cloud);
  descriptor = m2dp.Compute();

  std::ofstream myfile;
  myfile.open("cpp_desM2dp.txt");
  for (int i = 0; i < descriptor.cols; i++) {
    myfile << descriptor.at<double>(0, i) << ";\n";
  }
  myfile.close();


  /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }*/

  return 0;
}
