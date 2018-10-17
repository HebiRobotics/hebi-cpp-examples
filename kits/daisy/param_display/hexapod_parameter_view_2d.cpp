#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>

#include "hexapod_parameter_view_2d.hpp"
#include "robot/hexapod.hpp"

#include <iostream>

HexapodParameterView2D::HexapodParameterView2D(QGraphicsScene* scene) :
  _hexapod_view(scene)
{
}

void HexapodParameterView2D::resetTransforms()
{
  auto hexapod = hebi::Hexapod::createDummy(_params);

  Eigen::VectorXd angles;

  Eigen::VectorXd leg_vels = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd jacobian_ee = Eigen::MatrixXd::Zero(6, 3); // TODO: CHECK DIMENSION HERE! What does this affect?
  hebi::robot_model::MatrixXdVector jacobian_com;

  for (int i = 0; i < 6; ++i)
  {
    // Get positions from the hexapod class
    hexapod->getLeg(i)->setCmdStanceToHomeStance();
    hexapod->getLeg(i)->computeState(0, angles, leg_vels, jacobian_ee, jacobian_com);
    _hexapod_view.updateLeg(hexapod->getLeg(i), i, angles);
  }  
}


