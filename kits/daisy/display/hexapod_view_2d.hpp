#pragma once

#include <Eigen/Dense>

#include <QtCore/QObject>
#include <QEvent>

#include "hexapod_parameters.hpp"

class QGraphicsScene;
class QGraphicsLineItem;
class QGraphicsEllipseItem;
namespace hebi {
  class Leg;
}

class SetLineEvent : public QEvent
{
public:
  const static int type = (int)QEvent::User;
  SetLineEvent(QGraphicsLineItem* line_item, double x1, double y1, double x2, double y2)
    : QEvent(QEvent::Type(type))
  {
    this->line_item = line_item;
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
  }
  QGraphicsLineItem* line_item;
  double x1;
  double y1;
  double x2;
  double y2;
};

class SetEllipseRectEvent : public QEvent
{
public:
  const static int type = (int)QEvent::User + 1;
  SetEllipseRectEvent(QGraphicsEllipseItem* ellipse_item, double x1, double y1, double x2, double y2)
    : QEvent(QEvent::Type(type))
  {
    this->ellipse_item = ellipse_item;
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
  }
  QGraphicsEllipseItem* ellipse_item;
  double x1;
  double y1;
  double x2;
  double y2;
};

// TODO: do I need to inherit from QObject?
class HexapodView2D : public QObject
{
  Q_OBJECT

public:
  HexapodView2D(QGraphicsScene* scene);
  virtual ~HexapodView2D() noexcept = default;

  // Update the image using the current configuration of the given hexapod object.
  void updateLeg(const hebi::Leg* leg, int leg_index, const Eigen::VectorXd& angles);

  bool event(QEvent* ev);

private:
  QGraphicsEllipseItem* _body;

  QGraphicsLineItem* _to_shoulder[6];
  QGraphicsEllipseItem* _shoulder[6];

  QGraphicsLineItem* _shoulder_to_elbow[6];
  QGraphicsEllipseItem* _elbow[6];

  QGraphicsLineItem* _elbow_to_foot[6];

  Eigen::Matrix4d _view_tf;

  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
