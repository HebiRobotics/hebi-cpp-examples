#include <QGuiApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMessageBox>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtGui/QScreen>

#include "param_display/hexapod_parameter_view_2d.hpp"

#include "param_display/hexapod_parameter_gui.hpp"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
/*  Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
  view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
  QWidget *container = QWidget::createWindowContainer(view);
  QSize screenSize = view->screen()->size();
  container->setMinimumSize(QSize(200, 100));
  container->setMaximumSize(screenSize);
*/

  double overall_width = 900;
  double overall_height = 800;
  QGraphicsScene scene(QRectF(0, 0, overall_width, overall_height));
  HexapodParameterView2D* viewer = new HexapodParameterView2D(&scene);

  QGraphicsView view(&scene);
  view.setBackgroundBrush(Qt::black);
  view.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view.resize(overall_width + 10, overall_height + 10);
//  view.setMinimumSize(QSize(900, 800));
//  QSize screenSize = view.screen()->size();
//  view.setMaximumSize(screenSize);

  QWidget *widget = new QWidget;
  QHBoxLayout *hLayout = new QHBoxLayout(widget);
  QVBoxLayout *vLayout = new QVBoxLayout();
  vLayout->setAlignment(Qt::AlignTop);
  hLayout->addWidget(&view, 1);
  hLayout->addLayout(vLayout);

  // Manages the UI elements in the right panel
  HexapodParameterGui param_gui(viewer, vLayout);

  // (Try to) load default parameters
  param_gui.handleImportButton();

  // The 3D display
//  HexapodParameterView* param_view = param_gui.viewer();

  // Set root object of the scene
  //view->setRootEntity(rootEntity);

  // Show window
  //widget->show();
  widget->setWindowTitle(QStringLiteral("Set Hexapod Parameters"));
  widget->show();
  widget->resize(1200, 800);

  return app.exec();
}
