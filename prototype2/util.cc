//
// Created by leo on 01/05/18.
//

#include <boost/filesystem.hpp>

#include "util.h"

namespace pcl {
float LABF(float value) {
  if (value > 0.008856) return std::cbrtf(value);
  else return ((841.0/108.0)*value + 4.0/29.0);
}
void PointXYZRGBtoXYZLAB(PointXYZRGB &rgb, PointXYZLAB &lab) {
  float r = rgb.r / 255.0, g = rgb.g / 255.0, b = rgb.b / 255.0;
  r = (r > 0.04045) ? std::pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
  g = (r > 0.04045) ? std::pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
  b = (r > 0.04045) ? std::pow((b + 0.055) / 1.055, 2.4) : b / 12.92;

  r *= 100;
  g *= 100;
  b *= 100;

  float x, y, z;
  x = (r * 0.4124 + g * 0.3576 + b * 0.1805);
  y = (r * 0.2126 + g * 0.7152 + b * 0.0722);
  z = (r * 0.0193 + g * 0.1192 + b * 0.9505);

  float xo = 95.047, yo = 100, zo = 108.883;
  float fx = LABF(x / xo), fy = LABF(y / yo), fz = LABF(z / zo);
  lab.L = 116 * fy - 16;
  lab.a = 500 * (fx - fy);
  lab.b = 200 * (fy - fz);
}
}// namespace pcl

namespace prototype2 {

template<typename T>
T Util::min(const T *v, int n, int *ind) {
  //assert(n > 0);

  T min = v[0];
  if (ind != nullptr) *ind = 0;
  for (int i=1; i<n; i++)
    if (v[i] < min) {
      min = v[i];
      if (ind != nullptr) *ind=i;
    }

  return min;
}

void Util::AssertFileExists(std::string file) {
  if (!boost::filesystem::exists(file)) {
    file.append(" not found!");
    std::cerr << file << std::endl;
    throw std::exception();
  }
}

std::string Util::ZeroLeftPadding(int amount, int i) {
  return std::string((amount - (int) ((i < 10) ? 1 : log10(i) + 1)), '0').append(std::to_string(i));
}

template<typename T>
T Util::acos(T v) {
  if (v < -1.0)
    v = -1.0;
  else if (v > 1.0)
    v = 1.0;
  return std::acos(v);
}

Eigen::Vector3f Util::Sph2Cart(float azimuth, float elevation, float r) {
  float cos_azimuth, sin_azimuth, cos_elevation, sin_elevation;
  cos_azimuth = std::cos(azimuth);
  sin_azimuth = std::sin(azimuth);
  cos_elevation = std::cos(elevation);
  sin_elevation = std::sin(elevation);

  if (std::abs(cos_azimuth) < FLT_EPSILON) cos_azimuth = 0.f;
  if (std::abs(cos_elevation) < FLT_EPSILON) cos_elevation = 0.f;
  if (std::abs(sin_azimuth) < FLT_EPSILON) sin_azimuth = 0.f;
  if (std::abs(sin_elevation) < FLT_EPSILON) sin_elevation = 0.f;

  return Eigen::Vector3f(r * cos_elevation * cos_azimuth,
                         r * cos_elevation * sin_azimuth,
                         r * sin_elevation);
}

double Util::ConstrainAngle(double angle) {
    if (angle < 0)
        angle += 360;
    else if (angle > 360)
        angle -= 360;
    return angle;
}

template<>
pcl::PointXYZHSV Util::ConvertColor<pcl::PointXYZRGB, pcl::PointXYZHSV>(pcl::PointXYZRGB p1) {
  pcl::PointXYZHSV p2;
  cv::Mat hsv, rgb(1, 1, CV_32FC3, cv::Scalar(p1.r, p1.g, p1.b));
  cv::cvtColor(rgb, hsv, CV_RGB2HSV_FULL);
  p2.h = hsv.at<float>(0,0);
  p2.s = hsv.at<float>(0,1);
  p2.v = hsv.at<float>(0,2)/255.0;
  p2.x = p1.x;
  p2.y = p1.y;
  p2.z = p1.z;
  return p2;
}

template<>
pcl::PointXYZH Util::ConvertColor<pcl::PointXYZRGB, pcl::PointXYZH>(pcl::PointXYZRGB p1) {
  pcl::PointXYZH p2;

  return p2;
}

void Util::Rgb2Hsv(float r, float g, float b, float &h, float &s, float &v) {
  float K = 0.f;

  if (g < b) {
    std::swap(g, b);
    K = -1.f;
  }

  if (r < g) {
    std::swap(r, g);
    K = -2.f / 6.f - K;
  }

  float chroma = r - std::min(g, b);
  h = fabs(K + (g - b) / (6.f * chroma + 1e-20f));
  s = chroma / (r + 1e-20f);
  v = r;
}

double Util::L2norm(Eigen::Vector3d a, Eigen::Vector3d b) {
  return std::sqrt(std::pow(b(0) - a(0), 2) + std::pow(b(1) - a(1), 2) + std::pow(b(2) - a(2), 2));
}

float Util::PolarY(const float& a, const float& b) {
  return (std::sqrt(std::pow(a,2) + std::pow(b,2)));
}

float Util::PolarX(const float& a, const float& b) {
  return (std::atan2(a,b));
}

template int Util::min<int>(const int *v, int n, int *ind);
template float Util::acos<float>(float v);
template double Util::acos<double>(double v);
template pcl::PointXYZHSV Util::ConvertColor<pcl::PointXYZRGB, pcl::PointXYZHSV>(pcl::PointXYZRGB p1);
template pcl::PointXYZH Util::ConvertColor<pcl::PointXYZRGB, pcl::PointXYZH>(pcl::PointXYZRGB p1);




}//  namespace prototype2 {
