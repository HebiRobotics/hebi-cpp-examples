#include <QGuiApplication>

#include <Qt3DRender/qcamera.h>
#include <Qt3DRender/qcameralens.h>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtGui/QScreen>

#include <Qt3DRender/qpointlight.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qorbitcameracontroller.h>

#include "param_display/hexapod_parameter_view.hpp"

#include "param_display/hexapod_parameter_gui.hpp"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
  view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
  QWidget *container = QWidget::createWindowContainer(view);
  QSize screenSize = view->screen()->size();
  container->setMinimumSize(QSize(200, 100));
  container->setMaximumSize(screenSize);

  QWidget *widget = new QWidget;
  QHBoxLayout *hLayout = new QHBoxLayout(widget);
  QVBoxLayout *vLayout = new QVBoxLayout();
  vLayout->setAlignment(Qt::AlignTop);
  hLayout->addWidget(container, 1);
  hLayout->addLayout(vLayout);

  widget->setWindowTitle(QStringLiteral("Set Hexapod Parameters"));

  // Root entity
  Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

  // Camera
  Qt3DRender::QCamera *cameraEntity = view->camera();

  cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
  cameraEntity->setPosition(QVector3D(0.7, 1.0f, 0.7f));
  cameraEntity->setUpVector(QVector3D(0, 0, 1));
  cameraEntity->setViewCenter(QVector3D(0, 0, 0));

  Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
  Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
  light->setColor("white");
  light->setIntensity(1);
  lightEntity->addComponent(light);
  Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
  lightTransform->setTranslation(cameraEntity->position());
  lightEntity->addComponent(lightTransform);

  // For camera controls
  Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(rootEntity);
  camController->setCamera(cameraEntity);

  // Manages the UI elements in the right panel (created now to allow for callbacks)
  // TODO: rename? ParameterSliderPanel or something? Except it also starts the 3D viewer...
  HexapodParameterView* viewer = new HexapodParameterView(rootEntity);
  HexapodParameterGui param_gui(viewer, vLayout);

  // (Try to) load default parameters
  param_gui.handleImportButton();

  // The 3D display
  IHexapodParameterView* param_view = param_gui.viewer();

  // Set root object of the scene
  view->setRootEntity(rootEntity);

  // Show window
  widget->show();
  widget->resize(1200, 800);

  return app.exec();
}
