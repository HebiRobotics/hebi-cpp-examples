#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QCoreApplication>

#include "hexapod_view_2d.hpp"

#include "robot/leg.hpp"

#include <iostream>

HexapodView2D::HexapodView2D(QGraphicsScene* scene)
{
  _view_tf = Matrix4d::Identity();

  // Rotate
  double theta = 1.2;
  double t2 = -0.3;
  _view_tf << 1, 0, 0, 0,
              0, std::cos(theta), -std::sin(theta), 0,
              0, std::sin(theta), std::cos(theta), 0,
              0, 0, 0, 1;
  Matrix4d rot = Matrix4d::Identity();
  rot << std::cos(t2), -std::sin(t2), 0, 0,
         std::sin(t2), std::cos(t2), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
  _view_tf *= rot;
  
  // Scale!
  _view_tf.topLeftCorner<3,3>() *= 500;

  // Translate
  _view_tf(0,3) = 450;
  _view_tf(1,3) = 400;

  // Add grid:
  QPen grid_pen(QColor(105, 105, 105, 155), 1.0);
  int n = 10; // number of lines
  double s = .1; // spacing
  
  for (int i = -n; i <= n; ++i)
  {
    Vector4d a = _view_tf * Eigen::Vector4d(i * s, -n * s, 0, 1);
    Vector4d b = _view_tf * Eigen::Vector4d(i * s,  n * s, 0, 1);
    scene->addLine(a(0), a(1), b(0), b(1), grid_pen);

    a = _view_tf * Eigen::Vector4d(-n * s, i * s, 0, 1);
    b = _view_tf * Eigen::Vector4d( n * s, i * s, 0, 1);
    scene->addLine(a(0), a(1), b(0), b(1), grid_pen);
  }

  QPen fg_pen(QColor(255, 255, 255, 255), 2.0);

  _body = scene->addEllipse(0, 0, 0, 0, fg_pen); 
  for (int i = 0; i < 6; ++i)
  {
    _to_shoulder[i] = scene->addLine(0, 0, 0, 0, fg_pen);
    _shoulder[i] = scene->addEllipse(0, 0, 0, 0, fg_pen);
    _shoulder_to_elbow[i] = scene->addLine(0, 0, 0, 0, fg_pen);
    _elbow[i] = scene->addEllipse(0, 0, 0, 0, fg_pen);
    _elbow_to_foot[i] = scene->addLine(0, 0, 0, 0, fg_pen);
  }
}

bool HexapodView2D::event(QEvent* ev)
{
  if (ev->type() == SetLineEvent::type)
  {
    SetLineEvent* line_ev = static_cast<SetLineEvent*>(ev);
    line_ev->line_item->setLine(line_ev->x1, line_ev->y1, line_ev->x2, line_ev->y2);
    return true;
  }
  else if (ev->type() == SetEllipseRectEvent::type)
  {
    SetEllipseRectEvent* rect_ev = static_cast<SetEllipseRectEvent*>(ev);
    rect_ev->ellipse_item->setRect(rect_ev->x1, rect_ev->y1, rect_ev->x2, rect_ev->y2);
    return true;
  }
  // Make sure the rest of events are handled
  return QObject::event(ev);
}

void HexapodView2D::updateLeg(const hebi::Leg* leg, int leg_index, const Eigen::VectorXd& angles)
{
  QPen fg_pen(QColor(255, 255, 255, 255), 2.0);
  double r = 10; // Ellipse radius
 
  // TODO: move these to class member variables? At least only do the "set body" on construction!
  Eigen::Vector4d leg_base;
  Eigen::Vector4d leg_knee;
  Eigen::Vector4d leg_end;

  Eigen::Vector4d center;
  center << 0, 0, 0, 1;
  center = _view_tf * center;
//  _body->setRect(center(0) - r, center(1) - r, 2.0 * r, 2.0 * r);
  hebi::robot_model::Matrix4dVector frames;

  // Get positions from the leg
  leg->getKinematics().getFK(HebiFrameTypeOutput, angles, frames);
  leg_base = frames[0].topRightCorner<4,1>();
  leg_knee = frames[3].topRightCorner<4,1>();
  leg_end  = frames[5].topRightCorner<4,1>();

  // To shoulders
  Vector4d from = center;
  Vector4d to = _view_tf * leg_base;
  QEvent* evt = new SetLineEvent(_to_shoulder[leg_index], from(0), from(1), to(0), to(1));
  QCoreApplication::postEvent(this, evt);

  evt = new SetEllipseRectEvent(_shoulder[leg_index], to(0) - r, to(1) - r, 2.0 * r, 2.0 * r);
  QCoreApplication::postEvent(this, evt);

  // Thighs
  from = to;
  to = _view_tf * leg_knee;
  evt = new SetLineEvent(_shoulder_to_elbow[leg_index], from(0), from(1), to(0), to(1));
  QCoreApplication::postEvent(this, evt);

  // Elbows
  evt = new SetEllipseRectEvent(_elbow[leg_index], to(0) - r, to(1) - r, 2.0 * r, 2.0 * r);
  QCoreApplication::postEvent(this, evt);

  // Lower legs
  from = to;
  to = _view_tf * leg_end;
  evt = new SetLineEvent(_elbow_to_foot[leg_index], from(0), from(1), to(0), to(1));
  QCoreApplication::postEvent(this, evt);
}


