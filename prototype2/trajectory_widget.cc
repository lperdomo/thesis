#include "trajectory_widget.h"

TrajectoryWidget::TrajectoryWidget() {
  current_ = 0;
/*  layout_ = new QVBoxLayout(this);
  //scene_  = new QGraphicsScene();
  scene_  = new QGraphicsScene();
  distances_image_ = new QGraphicsItem();
  scene_->addItem(distances_image_);
  //scene_->add(distances_image_);
  //distance_matrix_ = new DistanceMatrixItem();
  //scene_->addItem(distance_matrix_);
  //QImage image = QImage(400, 400, QImage::Format_RGB888);
  //image.fill(Qt::black);
  //distance_matrix_ = scene_->addPixmap(QPixmap::fromImage(image));
  view_ = new QGraphicsView(scene_);
  layout_->addWidget(view_);*/
}

void TrajectoryWidget::paintEvent(QPaintEvent *event) {
  QPainter painter;
  painter.begin(this);
  if (trajectory_) {
    int cx = this->width()/2;
    int cy = this->height()/2;
    //double max_x = trajectory_->coeff(0, 0), min_x = trajectory_->coeff(0, 0);
    //double max_y = trajectory_->coeff(0, 2), min_y = trajectory_->coeff(0, 2);
    //std::exit(0);
    //for (int i = 0; i < trajectory_->rows(); i++) {
    //  if (trajectory_->coeff(i, 0) > max_x) max_x = trajectory_->coeff(i, 0);
    //  if (trajectory_->coeff(i, 2) > max_y) max_y = trajectory_->coeff(i, 2);
    //  if (trajectory_->coeff(i, 0) < min_x) min_x = trajectory_->coeff(i, 0);
    //  if (trajectory_->coeff(i, 2) < min_y) min_y = trajectory_->coeff(i, 2);
    //}
    //painter.setPen(QColor(255, 0, 0).rgba());
    //painter.drawRect(QRectF(0, 0, max_x+std::abs(min_x)+offset, max_y+std::abs(min_y)+offset));
    std::cout << "WWWWWWWWWWWWWWWW" << std::endl;
    for (int i = 0; i < current_; i++) {
      painter.setPen(QColor(0, 0, 0).rgba());
      painter.drawRect(QRectF(cx+trajectory_->coeff(i, 0), cy+trajectory_->coeff(i, 2), 2, 2));
    }
  }


  /*QImage image(distances_->rows(), distances_->cols(), QImage::Format_RGB32);
  QPainter painter;
  painter.begin(this);
  if (distances_->rows() > 0) {
    for (int i = 0; i < distances_->rows(); i++) {
      uchar *scan = image.scanLine(i);
      for (int j = 0; j < distances_->cols(); j++) {
        QRgb *rgbpixel = reinterpret_cast<QRgb *>(scan + j * 4);
        int gray = distances_->coeffRef(i, j) * 255;
        *rgbpixel = QColor(gray, gray, gray).rgba();
        //painter.setPen(QColor(gray, gray, gray).rgba());
        //painter.drawRect(i, j, 1, 1);
      }
    }
  } else {
    image.fill(Qt::black);
  }
  std::cout << "AEUHUAEHEAUAHEUAEHEAUAEHUAE" << std::endl;
  painter.drawImage(0, 0, image);
  painter.end();*/
}

void TrajectoryWidget::UpdateTrajectory(int n) {
  current_ = n;
  this->repaint();
  std::cout << "current_ widget " << current_ << std::endl;
}

void TrajectoryWidget::trajectory(boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > trajectory) {
  trajectory_ = trajectory;
}

//void Window::DrawDistances(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances) {
//QImage image(distances.rows(), distances.cols(), QImage::Format_RGB32);
//QImage image(distances.rows(), distances.cols(), QImage::Format_RGB32);
//std::cout << " " << distances.rows() << std::endl;
/*for (int i = 0; i < distances.rows(); i++) {
  for (int j = 0; j < distances.cols(); j++) {
    QColor c{255, 0, 0};
    image.setPixel(i, j, c.rgb());
    //output.at<uchar>(i, j) = static_cast<int>(std::floor(distances(i, j) * 255));
  }
}*/
//image_ = image;
//view_->viewport()->update();
//}

/*void Window::UpdateDistanceMatrix(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances) {
  if (distances.size() > 0) {
      std::cout << "epepepepepepeproiotr " << distances.rows() << " " << distances.cols() << std::endl;
      //QImage image(distances.rows(), distances.cols(), QImage::Format_RGB32);
      image_ = QImage(distances.rows(), distances.cols(), QImage::Format_RGB32) ;
      for (int i = 0; i < distances.rows(); i++) {
        for (int j = 0; j < distances.cols(); j++) {
          image_.setPixel(i, j, Qt::blue);
          //output.at<uchar>(i, j) = static_cast<int>(std::floor(distances(i, j) * 255));
        }
      }
      //distance_matrix_->image(image);
    //cv::Mat output(distances.rows(), distances.cols(), CV_8UC1);
    /*QPainter painter;
    QImage image(distances.rows(), distances.cols(), QImage::Format_RGB32);
    for (int i = 0; i < distances.rows(); i++) {
      for (int j = 0; j < distances.cols(); j++) {
        image.setPixel(i, j, Qt::blue);
        //output.at<uchar>(i, j) = static_cast<int>(std::floor(distances(i, j) * 255));
      }
    }
    painter.drawImage(QRect(100, 50, 100, 100), image);*/

//distance_matrix_->setPixmap(QPixmap::fromImage(image));

//cv::imshow("teste", output);
//cv::waitKey(10);
/*cv::cvtColor(output, output, CV_GRAY2RGB);
QImage image = QImage((uchar*) output.data, output.cols,
                      output.rows, output.step, QImage::Format_RGB888);
distance_matrix_->setPixmap(QPixmap::fromImage(image));
 */
//  }
//}
