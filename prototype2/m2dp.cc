//
// Created by phi on 21/05/18.
//

#include "m2dp.h"


//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

//#include <vtkRenderWindow.h>
//#include <vtkWindowToImageFilter.h>
//#include <vtkPNGWriter.h>


namespace prototype2 {

int M2dp::theta_bins       = 0;
int M2dp::rho_bins         = 0;
int M2dp::azimuth_angles   = 0;
int M2dp::elevation_angles = 0;

int CM2dp::color_bins   = 0;
int CM2dp::color_scheme = 0;

template<>
M2dpEstimation<M2dp>::M2dpEstimation() {}

template<>
M2dpEstimation<CM2dp>::M2dpEstimation() {
  color_stride_channel_1_ = CM2dp::theta_bins * CM2dp::rho_bins;
  color_stride_channel_2_ = color_stride_channel_1_ * 2;
  color_stride_channel_3_ = color_stride_channel_1_ * 3;
  if (CM2dp::color_scheme == static_cast<int>(ColorScheme::RGB)) {
    color_bins_ = 255.f / CM2dp::color_bins;
  } else if (CM2dp::color_scheme == static_cast<int>(ColorScheme::HSV)) {
    color_bins_ = 1.f / CM2dp::color_bins;
  } else if (CM2dp::color_scheme == static_cast<int>(ColorScheme::LAB)) {
    color_bins_ = 1.f / CM2dp::color_bins;
  }
  /** make sure all points in bins */
  color_bins_ += 0.001;
  for (int i = 0; i < 256; i++) {
    rgb_bins_lut_[i] = static_cast<int>(i / color_bins_);
  }
}

template<>
void M2dpEstimation<M2dp>::ExtractImpl(pcl::PointCloud<PointType>::ConstPtr cloud) {
  double time_start = pcl::getTime();
  //M2dp::theta_bins = 16;
  //M2dp::rho_bins = 8;
  //M2dp::azimuth_angles = 4;
  //M2dp::elevation_angles = 16;

  /** signature matrix A */
  Eigen::MatrixXd signature = Eigen::MatrixXd::Zero(M2dp::azimuth_angles * M2dp::elevation_angles,
                                                    M2dp::theta_bins * M2dp::rho_bins);
  //cv::Mat signature = cv::Mat(M2dp::azimuth_angles * M2dp::elevation_angles,
  //                            M2dp::theta_bins * M2dp::rho_bins, CV_64F);
  /** votes in bins */
  Eigen::ArrayXi votes(signature.cols());
  Eigen::ArrayXi color_votes;

  /** get the farthest point distance */
  float max_rho = PointCloudHelper::MaxDistance(PointCloudHelper::Centroid<PointType>(cloud), cloud);
  /** azimuth and elevation variations */
  auto azimuth_list                                = Eigen::ArrayXf::LinSpaced(M2dp::azimuth_angles, -M_PI_2, M_PI_2);
  auto elevation_list                              = Eigen::ArrayXf::LinSpaced(M2dp::elevation_angles, 0, M_PI_2);
  auto theta_list                                  = Eigen::ArrayXf::LinSpaced(M2dp::theta_bins+1, -M_PI, M_PI);
  Eigen::Array<float, 1, Eigen::Dynamic> rho_list  = Eigen::ArrayXf::LinSpaced(M2dp::rho_bins+1, 0, std::sqrt(max_rho)).pow(2);
  /** make sure all points in bins */
  rho_list.tail(1) += 0.001;
  /** PCA rotation invariant */
  pcl::PCA<PointType> pca;
  pca.setInputCloud(cloud);
  //Eigen::Matrix<double, 3, Eigen::Dynamic> data = pca.getCoefficients().template cast<double>();
  auto data = pca.getCoefficients();//.template cast<float>();
  //std::cout << pca.getEigenVectors() << std::endl;
  //std::cout << pca.getEigenValues() << std::endl;
  //std::ofstream file;
  //file.open("76.txt");
  //file << data.transpose();
  //file.close();
  long points = data.cols()-1;
  //std::cout << "eigenvectors " << pca.getEigenVectors() << std::endl;
  //só para comparar com o matlab
  //for (int i = 0; i < data.cols(); i++)
  //  data(2, i) = data(2, i)*-1;
  /** bin index of each point belonging to, ignore the centroid at the end */
  Eigen::ArrayXi data_bins(points);

  Eigen::Vector3f normal_vector, vector_c, projection_x, projection_y;
  float distance_h;
  /** plane index */
  int n = 1;
  for (int p = 0; p < M2dp::azimuth_angles; p++) {
    for (int q = 0; q < M2dp::elevation_angles; q++) {
      /** normal vector of the selected 2D plane */
      normal_vector = prototype2::Util::Sph2Cart(azimuth_list(p), elevation_list(q), 1);
      /** distance of vector [1,0,0] to the surface with normal vector */
      distance_h = Eigen::Vector3f::UnitX().dot(normal_vector);
      /** a new vector, c = h*vecN, so that vector [1,0,0]-c is the
        * projection of x-axis onto the plane with normal vector */
      vector_c = distance_h*normal_vector;
      /** x-axis - c, the projection */
      projection_x = Eigen::Vector3f::UnitX()-vector_c;
      /** given the normal vector vecN and the projected x-axis px, the
        * y-axis is cross(vecN,px) */
      projection_y = normal_vector.cross(projection_x);
      /** projection of data onto space span{px,py}
        * pdata = [data*px' data*py']; */
      //Eigen::Array<float, 1, Eigen::Dynamic> data_projection_x = projection_x.transpose()*data;
      auto data_projection_x = projection_x.transpose()*data;
      //Eigen::Array<double, 1, Eigen::Dynamic> data_projection_y = projection_y.transpose()*data;
      //Eigen::Array<float, 1, Eigen::Dynamic> data_projection_y = projection_y.transpose()*data;
      auto data_projection_y = projection_y.transpose()*data;
      /** represent data in polar coordinates
        * [theta,rho] = cart2pol(pdata(:,1),pdata(:,2)); */
      /*Eigen::Array<double, 1, Eigen::Dynamic> theta =
        data_projection_y.binaryExpr(data_projection_x, [] (double a, double b) { return std::atan2(a,b);} );
      Eigen::Array<double, 1, Eigen::Dynamic> rho =
        data_projection_x.binaryExpr(data_projection_y, [] (double a, double b) { return std::sqrt(std::pow(a,2)
                                                                                                   + std::pow(b,2));} );*/
      const Eigen::Array<float, 1, Eigen::Dynamic> rho = data_projection_x.binaryExpr(data_projection_y, PolarY());
      const Eigen::Array<float, 1, Eigen::Dynamic> theta = data_projection_y.binaryExpr(data_projection_x, PolarX());

      /** reset votes */
      votes = Eigen::ArrayXi::Zero(points);
      /** count points in bins */
      int idxR, idxT;

      for (int i = 0; i < points; i++) {
        /** find which theta bin this point belonging to */
        idxT = M2dp::theta_bins-1;
        //std::cout << "x " << data_projection_x(i) << std::endl;
        //std::cout << "y " << data_projection_y(i) << std::endl;
        /** represent data in polar coordinates
          * [theta,rho] = cart2pol(pdata(:,1),pdata(:,2)); */
        //float theta = Util::PolarY(data_projection_y(i), data_projection_x(i));
        //float rho = Util::PolarX(data_projection_x(i), data_projection_y(i));
        //std::cout << "theta " << theta << std::endl;
        //std::cout << "rho " << rho << std::endl;
        for (int t = 0; t < M2dp::theta_bins-1; t++) {
          if (theta(i) < theta_list(t+1)) {
          //if (theta < theta_list(t+1)) {
            idxT = t;
            break;
          }
        }
        /** find which radius bin this point belonging to */
        idxR = M2dp::rho_bins-1;
        for (int r = 0; r < M2dp::rho_bins-1; r++) {
          //if (rho < rho_list(r+1)) {
          if (rho(i) < rho_list(r+1)) {
            idxR = r;
            break;
          }
        }
        /** record which bin this point in */
        data_bins(i) = idxR * M2dp::theta_bins + idxT; //base do circulo + bin do circulo
        votes(data_bins(i))++;
      }
      /** normalize counts */
      for (int i = 0; i < signature.cols(); i++) {
        /** record the signature of the n-th plane */
        signature(n-1, i) = (double) (votes(i))/(points);
        //signature.at<double>(n-1, i) = (double) (votes(i))/(points);
      }
      n++;
    }
    //std::cout << "elevation loop took " << double(pcl::getTime() - time_start_2) << std::endl;
  }
  //std::ofstream file;
  //file.open("75_signature.txt");
  //file << signature;
  //file.close();
  /** run SVD on A and use [u1,v1] as the final output */
  //cv::SVD::compute(signature);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(signature, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //std::cout << svd. << std::endl;
  //std::cout << svd.getEigenValues() << std::endl;
  /** "The singular vectors sign can be determined from the sign of the inner product
   * of the singular vector and the individual data vectors. The data vectors may have different
   * orientation but it then make intuitive as well as practical sense to choose the direction in which
   * the majority of the vectors point"
   * https://prod.sandia.gov/techlib-noauth/access-control.cgi/2007/076422.pdf */
  //cv::Mat descriptor(1, svd.matrixU().rows() + svd.matrixV().rows(), CV_64F);
  result_ = cv::Mat(1, svd.matrixU().rows() + svd.matrixV().rows(), CV_32F);
  int u, v, sign = 1;
  for (u = 0; u < svd.matrixU().rows(); u++) {
    if (u == 0 && (svd.matrixU())(u, 0) < 0) sign = -1;
    result_.at<float>(0, u) = (svd.matrixU())(u, 0)*sign;
  }
  for (v = 0; v < svd.matrixV().rows(); v++) {
    result_.at<float>(0, u+v) = (svd.matrixV())(v, 0)*sign;
  }

  //std::cout << "M2DP estimation took " << double(pcl::getTime() - time_start) << std::endl;
  //return descriptor;
}

template<>
void M2dpEstimation<CM2dp>::ExtractImpl(pcl::PointCloud<PointType>::ConstPtr cloud) {
  double time_start = pcl::getTime();
  //M2dp::theta_bins = 16;
  //M2dp::rho_bins = 8;
  //M2dp::azimuth_angles = 4;
  //M2dp::elevation_angles = 16;
  /** signature matrix A */
  Eigen::MatrixXd signature = Eigen::MatrixXd::Zero(CM2dp::azimuth_angles * CM2dp::elevation_angles,
                                                    CM2dp::theta_bins * CM2dp::rho_bins +
                                                    CM2dp::color_bins * CM2dp::rho_bins * 3);

  Eigen::Array<int, Eigen::Dynamic, 3> cloud_color_scheme(cloud->points.size(), 3);
  /** votes in bins */
  Eigen::ArrayXi votes(signature.cols());
  /** get the farthest point distance */
  float max_rho = PointCloudHelper::MaxDistance(PointCloudHelper::Centroid<PointType>(cloud), cloud);
  /** azimuth and elevation variations */
  auto azimuth_list                               = Eigen::ArrayXf::LinSpaced(CM2dp::azimuth_angles, -M_PI_2, M_PI_2);
  auto elevation_list                             = Eigen::ArrayXf::LinSpaced(CM2dp::elevation_angles, 0, M_PI_2);
  auto theta_list                                 = Eigen::ArrayXf::LinSpaced(CM2dp::theta_bins+1, -M_PI, M_PI);
  Eigen::Array<float, 1, Eigen::Dynamic> rho_list = Eigen::ArrayXf::LinSpaced(CM2dp::rho_bins+1, 0, std::sqrt(max_rho)).pow(2);
  /** make sure all points in bins */
  rho_list.tail(1) += 0.001;
  /** PCA rotation invariant */
  pcl::PCA<PointType> pca;
  pca.setInputCloud(cloud);
  //Eigen::Matrix<double, 3, Eigen::Dynamic> data = pca.getCoefficients().template cast<double>();
  auto data = pca.getCoefficients();//.template cast<float>();
  //std::cout << pca.getEigenVectors() << std::endl;
  //std::cout << pca.getEigenValues() << std::endl;
  //std::ofstream file;
  //file.open("76.txt");
  //file << data.transpose();
  //file.close();
  long points = data.cols()-1;
  //std::cout << "eigenvectors " << pca.getEigenVectors() << std::endl;
  //só para comparar com o matlab
  //for (int i = 0; i < data.cols(); i++)
  //  data(2, i) = data(2, i)*-1;
  /** bin index of each point belonging to, ignore the centroid at the end */
  Eigen::ArrayXi data_bins(points);

  //std::cout << "pca took " << double(pcl::getTime() - time_start) << std::endl;

  Eigen::Vector3f normal_vector, vector_c, projection_x, projection_y;
  float distance_h;
  /** plane index */
  int n = 1;
  for (int p = 0; p < CM2dp::azimuth_angles; p++) {
    for (int q = 0; q < CM2dp::elevation_angles; q++) {
      /** normal vector of the selected 2D plane */
      normal_vector = prototype2::Util::Sph2Cart(azimuth_list(p), elevation_list(q), 1);
      /** distance of vector [1,0,0] to the surface with normal vector */
      distance_h = Eigen::Vector3f::UnitX().dot(normal_vector);
      /** a new vector, c = h*vecN, so that vector [1,0,0]-c is the
        * projection of x-axis onto the plane with normal vector */
      vector_c = distance_h*normal_vector;
      /** x-axis - c, the projection */
      projection_x = Eigen::Vector3f::UnitX()-vector_c;
      /** given the normal vector vecN and the projected x-axis px, the
        * y-axis is cross(vecN,px) */
      projection_y = normal_vector.cross(projection_x);

      //std::cout << "partial 1 took " << double(pcl::getTime() - time_start_2) << std::endl;
      /** projection of data onto space span{px,py}
        * pdata = [data*px' data*py']; */
      auto data_projection_x = projection_x.transpose()*data;
      auto data_projection_y = projection_y.transpose()*data;        
      //Eigen::Array<double, 1, Eigen::Dynamic> data_projection_x = projection_x.transpose()*data;
      //Eigen::Matrix<float, 1, Eigen::Dynamic> data_projection_x = projection_x.transpose()*data;

      //std::cout << "partial 2 took " << double(pcl::getTime() - time_start_2) << std::endl;

      //Eigen::Array<double, 1, Eigen::Dynamic> data_projection_y = projection_y.transpose()*data;
      //Eigen::Matrix<float, 1, Eigen::Dynamic> data_projection_y = projection_y.transpose()*data;

      //std::cout << "partial 3 took " << double(pcl::getTime() - time_start_2) << std::endl;
      /** represent data in polar coordinates
        * [theta,rho] = cart2pol(pdata(:,1),pdata(:,2)); */
      const Eigen::Array<float, 1, Eigen::Dynamic> rho = data_projection_x.binaryExpr(data_projection_y, PolarY());
      const Eigen::Array<float, 1, Eigen::Dynamic> theta = data_projection_y.binaryExpr(data_projection_x, PolarX());        
      //Eigen::Array<double, 1, Eigen::Dynamic> theta =
      //  data_projection_y.binaryExpr(data_projection_x, [] (double a, double b) { return std::atan2(a,b);} );
      //Eigen::Array<double, 1, Eigen::Dynamic> rho =
      //  data_projection_x.binaryExpr(data_projection_y, [] (double a, double b) { return std::sqrt(std::pow(a,2)
      //                                                                                            + std::pow(b,2));} );
      /*const Eigen::Array<double, 1, Eigen::Dynamic> rho = data_projection_x.binaryExpr(data_projection_y, PolarY());

      std::cout << "partial 4 took "
                << double(pcl::getTime() - time_start_2)
                << std::endl;

      const Eigen::Array<double, 1, Eigen::Dynamic> theta = data_projection_y.binaryExpr(data_projection_x, PolarX());

      std::cout << "partial 5 took "
                << double(pcl::getTime() - time_start_2)
                << std::endl;*/

      //cv::Mat rho, theta, dpx, dpy;
      //cv::eigen2cv(data_projection_x, dpx);
      //cv::eigen2cv(data_projection_y, dpy);

      //cv::cartToPolar(dpx, dpy, rho, theta, false);

      //std::cout << "partial 5 took " << double(pcl::getTime() - time_start_2)  << std::endl;
      /** reset votes */
      votes = Eigen::ArrayXi::Zero(signature.cols());

      //std::cout << "partial 6 took " << double(pcl::getTime() - time_start_2) << std::endl;

      /** count points in bins */
      int idxR, idxT;
      for (int i = 0; i < points; i++) {
        /** find which theta bin this point belonging to */
        idxT = CM2dp::theta_bins-1;
        /** represent data in polar coordinates
          * [theta,rho] = cart2pol(pdata(:,1),pdata(:,2)); */
        //float theta = Util::PolarX(data_projection_y(i), data_projection_x(i));
        //float rho = Util::PolarY(data_projection_x(i), data_projection_y(i));
        for (int t = 0; t < CM2dp::theta_bins-1; t++) {
          //if (theta.at<double>(0, i) < theta_list(t+1)) {
          if (theta(i) < theta_list(t+1)) {
          //if (theta < theta_list(t+1)) {
            idxT = t;
            break;
          }
        }
        /** find which radius bin this point belonging to */
        idxR = CM2dp::rho_bins-1;
        for (int r = 0; r < CM2dp::rho_bins-1; r++) {
          //if (rho.at<double>(0, i) < rho_list(r+1)) {
          if (rho(i) < rho_list(r+1)) {
          //if (rho < rho_list(r+1)) {
            idxR = r;
            break;
          }
        }

        /** record which bin this point in */
        if (n == 1) {
          pcl::PointXYZRGB rgb = cloud->points[i];
          if (CM2dp::color_scheme == static_cast<int>(ColorScheme::RGB)) {
            cloud_color_scheme(i, 0) = static_cast<int>(rgb.r / color_bins_);
            cloud_color_scheme(i, 1) = static_cast<int>(rgb.g / color_bins_);
            cloud_color_scheme(i, 2) = static_cast<int>(rgb.b / color_bins_);
          } else if (CM2dp::color_scheme == static_cast<int>(ColorScheme::HSV)) {
            pcl::PointXYZHSV hsv;
            pcl::PointXYZRGBtoXYZHSV(rgb, hsv);
            cloud_color_scheme(i, 0) = static_cast<int>((hsv.h/360) / color_bins_);
            cloud_color_scheme(i, 1) = static_cast<int>(hsv.s / color_bins_);
            cloud_color_scheme(i, 2) = static_cast<int>(hsv.v / color_bins_);
          } else if (CM2dp::color_scheme == static_cast<int>(ColorScheme::LAB)) {
            pcl::PointXYZLAB lab;
            pcl::PointXYZRGBtoXYZLAB(rgb, lab);
            cloud_color_scheme(i, 0) = static_cast<int>((lab.L/100) / color_bins_);
            cloud_color_scheme(i, 1) = static_cast<int>(((lab.a+128)/255) / color_bins_);
            cloud_color_scheme(i, 2) = static_cast<int>(((lab.b+128)/255) / color_bins_);
          }
        }

        int rho_color_stride = idxR * CM2dp::color_bins;
        data_bins(i) = idxR * CM2dp::theta_bins + idxT;
        votes(data_bins(i))++;
        votes(color_stride_channel_1_ + rho_color_stride + cloud_color_scheme(i, 0))++;
        votes(color_stride_channel_2_ + rho_color_stride + cloud_color_scheme(i, 1))++;
        votes(color_stride_channel_3_ + rho_color_stride + cloud_color_scheme(i, 2))++;
      }
      //std::cout << "partial 7 took " << double(pcl::getTime() - time_start_2) << std::endl;
      /** normalize counts */
      for (int i = 0; i < signature.cols(); i++) {
        /** record the signature of the n-th plane */
        signature(n-1, i) = (double) (votes(i))/(points);
      }
      //std::cout << "partial 8 took " << double(pcl::getTime() - time_start_2) << std::endl;
      //std::exit(0);
      n++;
    }
    //std::cout << "elevation loop took " << double(pcl::getTime() - time_start_2) << std::endl;
  }
  /** run SVD on A and use [u1,v1] as the final output */
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(signature, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //cv::Mat descriptor(1, svd.matrixU().rows() + svd.matrixV().rows(), CV_64F);
  result_ = cv::Mat(1, svd.matrixU().rows() + svd.matrixV().rows(), CV_32F);
  int u, v, sign = 1;
  for (u = 0; u < svd.matrixU().rows(); u++) {
    if (u == 0 && (svd.matrixU())(u, 0) < 0) sign = -1;
    result_.at<float>(0, u) = (svd.matrixU())(u, 0)*sign;
  }
  for (v = 0; v < svd.matrixV().rows(); v++) {
    result_.at<float>(0, u+v) = (svd.matrixV())(v, 0)*sign;
  }
  //std::cout << "mU " << svd.matrixU().rows() << std::endl;
  //std::cout << "mV " << svd.matrixV().rows() << std::endl;
  //std::exit(0);
  //std::cout << "CM2DP estimation took " << double(pcl::getTime() - time_start) << std::endl;
  //return descriptor;
}

/*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr teste(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*input_cloud_, *teste);

    for (int i = 0; i < points; i++) {
      if (data_bins(n-1, i) < 16) {
        if (data_bins(n-1, i)%2 == 0) {
          teste->points[i].r = 27;
          teste->points[i].g = 158;
          teste->points[i].b = 119;
        } else {
          teste->points[i].r = 14;
          teste->points[i].g = 77;
          teste->points[i].b = 57;
        }
      } else if (data_bins(n-1, i) < 32) {
        if (data_bins(n-1, i)%2 != 0) {
          teste->points[i].r = 217;
          teste->points[i].g = 95;
          teste->points[i].b = 2;
        } else {
          teste->points[i].r = 77;
          teste->points[i].g = 34;
          teste->points[i].b = 1;
        }
      } else if (data_bins(n-1, i) < 48) {
        if (data_bins(n-1, i)%2 == 0) {
          teste->points[i].r = 117;
          teste->points[i].g = 112;
          teste->points[i].b = 179;
        } else {
          teste->points[i].r = 50;
          teste->points[i].g = 48;
          teste->points[i].b = 77;
        }
      } else if (data_bins(n-1, i) < 64) {
        if (data_bins(n-1, i)%2 != 0) {
          teste->points[i].r = 231;
          teste->points[i].g = 41;
          teste->points[i].b = 138;
        } else {
          teste->points[i].r = 77;
          teste->points[i].g = 14;
          teste->points[i].b = 45;
        }
      } else if (data_bins(n-1, i) < 80) {
        if (data_bins(n-1, i)%2 == 0) {
          teste->points[i].r = 102;
          teste->points[i].g = 166;
          teste->points[i].b = 30;
        } else {
          teste->points[i].r = 47;
          teste->points[i].g = 77;
          teste->points[i].b = 14;
        }
      } else if (data_bins(n-1, i) < 96) {
        if (data_bins(n-1, i)%2 != 0) {
          teste->points[i].r = 230;
          teste->points[i].g = 171;
          teste->points[i].b = 2;
        } else {
          teste->points[i].r = 77;
          teste->points[i].g = 57;
          teste->points[i].b = 1;
        }
      } else if (data_bins(n-1, i) < 112) {
        if (data_bins(n-1, i)%2 == 0) {
          teste->points[i].r = 166;
          teste->points[i].g = 118;
          teste->points[i].b = 29;
        } else {
          teste->points[i].r = 77;
          teste->points[i].g = 56;
          teste->points[i].b = 13;
        }
      } else if (data_bins(n-1, i) < 128) {
        if (data_bins(n-1, i)%2 != 0) {
          teste->points[i].r = 102;
          teste->points[i].g = 102;
          teste->points[i].b = 102;
        } else {
          teste->points[i].r = 51;
          teste->points[i].g = 51;
          teste->points[i].b = 51;
        }
      }
      teste->points[i].x = data_projection_x(i);
      teste->points[i].y = data_projection_y(i);
      teste->points[i].z = 0.f;
    }
//}


  pcl::visualization::PointCloudColorHandlerRGBField<PointType> color(teste);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZRGB> (teste, color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
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
  viewer->setCameraParameters(camera);


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
*/

} // namespace prototype2
