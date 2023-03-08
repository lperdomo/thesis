#include "distances_widget.h"

DistancesWidget::DistancesWidget() {
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

void DistancesWidget::paintEvent(QPaintEvent *event) {
  QImage image(distances_->rows(), distances_->cols(), QImage::Format_RGB32);
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
  painter.end();
}

void DistancesWidget::distances(boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic>> distances) {
  distances_ = distances;
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
