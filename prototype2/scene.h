//
// Created by phi on 15/10/18.
//

#ifndef PROTOTYPE2_SCENE_H
#define PROTOTYPE2_SCENE_H

#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QPainter>
#include <QRectF>
#include <QKeyEvent>

#include "util.h"

class DistanceMatrixItem : public QGraphicsItem {
public:
  DistanceMatrixItem();
  QRectF boundingRect() const;
  void image(QImage image);
protected:
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0);
private:
  QImage image_;
};


#endif //PROTOTYPE2_SCENE_H
