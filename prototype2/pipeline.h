//
// Created by phi on 11/04/18.
//

#ifndef PROTOTYPE2_PIPELINE_H
#define PROTOTYPE2_PIPELINE_H

#include <iostream>
#include <QApplication>
#include <QTimer>
#include <QThread>

#include "dataset.h"
#include "m2dp.h"
#include "shot.h"
#include "matcher.h"
#include "output.h"
#include "distances_widget.h"
#include "trajectory_widget.h"

namespace prototype2 {

class Pipeline : public QObject {
  Q_OBJECT
public:
  typedef boost::shared_ptr<Pipeline> Ptr;
  enum class Task : int {
    END    = 0,
    BUILD  = 1,
    QUERY  = 2,
    MATRIX = 3,
    PR     = 4,
    MATLAB = 5
  };
  Pipeline(int argc, char **argv);
  void Run();
  void task(int code);
  void descriptor(Descriptor::Ptr descriptor);
  void matcher(Matcher::Ptr matcher);
  Descriptor::Ptr descriptor();
  Matcher::Ptr matcher();
  Dataset& query();
  Dataset& database();
  Output output_;
public slots:
  void Matrix();
  //void MatrixOutput(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>&);
signals:
  void TaskMatrix();
  void ShowMatrix();
  void ShowCurrentTrajectory(int);
  //void ShowMatrix(int, int);
  //void HasNewMatrix(const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>&);
private:
  void Build();
  void Query();
  void PrecisionRecall();
  void Matlab();
  void QueryOutput();
  Task task_;
  Dataset query_;
  Dataset database_;
  Descriptor::Ptr descriptor_;
  Matcher::Ptr matcher_;
  int current_;
  //DistancesWidget distances_widget_;
  //TrajectoryWidget trajectory_widget_;
  //QThread *thread_distances_;
};

} // namespace prototype2

#endif //PROTOTYPE2_PIPELINE_H
