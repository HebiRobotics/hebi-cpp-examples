#pragma once

#include <Eigen/Dense>

#include <QtCore/QObject>

#include "i_hexapod_parameter_view.hpp"
#include "display/hexapod_view_2d.hpp"
#include "hexapod_parameters.hpp"

class QGraphicsScene;
class QGraphicsLineItem;
class QGraphicsEllipseItem;

class HexapodParameterView2D : public IHexapodParameterView
{
  Q_OBJECT

public:
  HexapodParameterView2D(QGraphicsScene* scene);
  virtual ~HexapodParameterView2D() noexcept = default;

private:
  HexapodView2D _hexapod_view;

  void resetTransforms() override;
};
