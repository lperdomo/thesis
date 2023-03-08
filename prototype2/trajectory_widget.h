#ifndef TRAJECTORY_WIDGET_H
#define TRAJECTORY_WIDGET_H

#include <QGraphicsView>
#include <QImage>

#include "util.h"
#include "scene.h"

class TrajectoryWidget : public QWidget {
  Q_OBJECT
public:
  TrajectoryWidget();
//public slots:
  //void DrawDistances(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances);
  //void DrawDistances(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances);
  //void UpdateDistanceMatrix(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances);
  void trajectory(boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > trajectory);
public slots:
  void UpdateTrajectory(int n);
protected:
  void paintEvent(QPaintEvent *event) override;
private:
  //QVBoxLayout* layout_;
  //QGraphicsScene* scene_;
  //QGraphicsView* view_;
  int current_;
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > trajectory_;
};

#endif //TRAJECTORY_WIDGET_H
