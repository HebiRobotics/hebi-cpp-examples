#pragma once

#include <memory>
#include <QtCore/QObject> // For ParameterManager
#include <Qt3DCore/QEntity>
#include <QtWidgets/QSlider>

#include "i_hexapod_parameter_view.hpp"

class QVBoxLayout;

// Parameter manager for Qt compatibility
class HexapodParameterGui : public QObject
{
  Q_OBJECT

public:
  // First argument is root of 3D scene, second is the layout for sliders/etc.
  HexapodParameterGui(IHexapodParameterView* /*Qt3DCore::QEntity* */, QVBoxLayout*);
  virtual ~HexapodParameterGui() noexcept = default;

  IHexapodParameterView* viewer() { return viewer_.get(); }

public slots:
  void handleImportButton();
  void handleExportButton();

private:
  std::unique_ptr<IHexapodParameterView> viewer_;

  QSlider* stance_radius_slider_;
  QSlider* body_height_slider_;
};

