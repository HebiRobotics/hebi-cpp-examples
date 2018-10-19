#pragma once

#include <QtCore/QObject>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include "robot/hexapod_parameters.hpp"
#include "i_hexapod_parameter_view.hpp"

class HexapodParameterView : public IHexapodParameterView
{
  Q_OBJECT

public:
  explicit HexapodParameterView(Qt3DCore::QEntity *rootEntity);
  ~HexapodParameterView();

  void resetTransforms() override;

private:
  Qt3DCore::QTransform* _body_tf;
  Qt3DCore::QEntity* _body;

  Qt3DCore::QEntity* _to_shoulder[6];
  Qt3DExtras::QCylinderMesh* _to_shoulder_mesh[6];
  Qt3DCore::QTransform* _to_shoulder_tf[6];

  Qt3DCore::QEntity* _shoulder[6];
  Qt3DCore::QTransform* _shoulder_tf[6];

  Qt3DCore::QEntity* _shoulder_to_elbow[6];
  Qt3DExtras::QCylinderMesh* _shoulder_to_elbow_mesh[6];
  Qt3DCore::QTransform* _shoulder_to_elbow_tf[6];

  Qt3DCore::QEntity* _elbow[6];
  Qt3DCore::QTransform* _elbow_tf[6];

  Qt3DCore::QEntity* _elbow_to_foot[6];
  Qt3DExtras::QCylinderMesh* _elbow_to_foot_mesh[6];
  Qt3DCore::QTransform* _elbow_to_foot_tf[6];

  Qt3DCore::QEntity *_root;
};

