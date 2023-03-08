#ifndef DISTANCES_VIEW_H
#define DISTANCES_VIEW_H

#include <QGraphicsView>
#include <QImage>

#include "util.h"
#include "scene.h"

class DistancesWidget : public QWidget {
  Q_OBJECT
public:
  DistancesWidget();
//public slots:
  //void DrawDistances(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances);
  //void DrawDistances(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances);
  //void UpdateDistanceMatrix(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>& distances);
  void distances(boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic>> distances);
  void Teste(int x, int y);
protected:
  void paintEvent(QPaintEvent *event) override;
private:
  //QVBoxLayout* layout_;
  //QGraphicsScene* scene_;
  //QGraphicsView* view_;
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic>> distances_;
};

#endif //DISTANCES_VIEW_H
