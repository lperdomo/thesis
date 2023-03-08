//
// Created by leo on 01/05/18.
//

#ifndef PROTOTYPE2_UTIL_H
#define PROTOTYPE2_UTIL_H

#include <iostream>

#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/common/time.h>

namespace pcl {
struct PointXYZH {
  PCL_ADD_POINT4D
  union {
    struct {
      float h;
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
struct PointXYZLAB {
  PCL_ADD_POINT4D
  union {
    struct {
      float L;
      float a;
      float b;
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
struct PointXYZAB {
  PCL_ADD_POINT4D
  union {
    struct {
      float a;
      float b;
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
float LABF(float value);
void PointXYZRGBtoXYZLAB(PointXYZRGB &rgb, PointXYZLAB &lab);
}// namespace pcl

namespace prototype2 {

typedef pcl::PointXYZRGB PointTypeInput;
typedef pcl::PointXYZRGB PointType;

enum class ColorScheme : int {
  RGB = 0,
  HSV = 1,
  LAB = 2,
  AB  = 3
};

class Util {
public:
  template<typename T>
  T static min(const T *v, int n, int *ind=nullptr);
  template<typename T>
  T static acos(const T v);
  float static PolarX(const float& a, const float& b);
  float static PolarY(const float& a, const float& b);
  Eigen::Vector3f static Sph2Cart(float azimuth, float elevation, float r);
  std::string static ZeroLeftPadding(int amount, int i);
  void static AssertFileExists(std::string file);
  double static ConstrainAngle(double angle);
  void static Rgb2Hsv(float r, float g, float b, float &h, float &s, float &v);
  double static L2norm(Eigen::Vector3d a, Eigen::Vector3d b);
  template<class T, class T2>
  T2 static ConvertColor(T p1);
};

}// namespace prototype2


#endif //PROTOTYPE2_UTIL_H
