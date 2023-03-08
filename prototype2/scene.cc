//
// Created by phi on 15/10/18.
//

#include "scene.h"

DistanceMatrixItem::DistanceMatrixItem() {

}

QRectF DistanceMatrixItem::boundingRect() const {
  return QRectF(1,1,1,1);
}

void DistanceMatrixItem::image(QImage image) {
  image_ = image;
}

void DistanceMatrixItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
  painter->drawImage(QRect(100, 50, 100, 100), image_);
}
