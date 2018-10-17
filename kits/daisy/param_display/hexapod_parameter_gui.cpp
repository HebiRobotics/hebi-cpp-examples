#include "hexapod_parameter_gui.hpp"
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

#include "hexapod_parameters.hpp"

QSlider* addSlider()
{
  QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
  slider->setTickInterval(10);
  slider->setTickPosition(QSlider::TicksBelow);
  slider->setSingleStep(1);
  slider->setMinimum(0);
  slider->setMaximum(100);
  slider->setTracking(true);
  return slider;
}

HexapodParameterGui::HexapodParameterGui(IHexapodParameterView* viewer/* Qt3DCore::QEntity* root*/, QVBoxLayout* layout)

//QSlider* stance_radius_slider, QSlider* body_height_slider) :
//  stance_radius_slider_(stance_radius_slider), body_height_slider_(body_height_slider)
{
  // Create the UI

  // 3D viewer panel
  // Start with default parameters // TODO: remove requirement to pass parameters in to 3D viewer!
  hebi::HexapodParameters init_params;
//  viewer_.reset(new HexapodParameterView(root, init_params));
  viewer_.reset(viewer);

  // Title
  QLabel* title = new QLabel("HeXapod Parameters");
  auto title_font = title->font();
  title_font.setWeight(QFont::Bold);
  title->setFont(title_font);

  // Add sliders and labels
  QLabel* stance_radius_label = new QLabel("Stance Radius");
  stance_radius_slider_ = addSlider();

  QLabel* body_height_label = new QLabel("Body Height");
  body_height_slider_ = addSlider();

  // Add buttons
  QPushButton* import_button = new QPushButton("Reload From File");
  QPushButton* export_button = new QPushButton("Save To File");
  QPushButton* log_dir_button = new QPushButton("Choose Log Folder");

  // Connect sliders and buttons 
  QObject::connect(stance_radius_slider_, &QSlider::valueChanged,
                   viewer_.get(), &IHexapodParameterView::stanceRadiusChanged);
  QObject::connect(body_height_slider_, &QSlider::valueChanged,
                   viewer_.get(), &IHexapodParameterView::bodyHeightChanged);

  QObject::connect(import_button, &QPushButton::released,
                   this, &HexapodParameterGui::handleImportButton);
  QObject::connect(export_button, &QPushButton::released,
                   this, &HexapodParameterGui::handleExportButton);
  QObject::connect(log_dir_button, &QPushButton::released,
                   this, &HexapodParameterGui::handleLogDirButton);

  // Add widgets
  layout->addWidget(title);
  layout->addWidget(stance_radius_label);
  layout->addWidget(stance_radius_slider_);
  layout->addWidget(body_height_label);
  layout->addWidget(body_height_slider_);
  layout->addWidget(log_dir_button);
//  QLabel* label = new QLabel(tr(""));
//  layout->addWidget(label);
  // TODO: add some space here! Divider?
  layout->addWidget(import_button);
  layout->addWidget(export_button);
}

void HexapodParameterGui::handleImportButton()
{
  hebi::HexapodParameters params;
  if (!params.loadFromFile("hex_config.xml"))
  {
    params.resetToDefaults();
    QMessageBox param_error;
    param_error.setWindowTitle("Unable to load parameters");
    param_error.setText("Error locating or parsing parameter file!  Loading default parameters.");
    param_error.exec();
  }

  viewer_->paramsUpdated(params);
  viewer_->resetTransforms();
  // Update sliders
  stance_radius_slider_->setValue(viewer_->convertStanceRadiusToSlider(viewer_->params()->stance_radius_));
  body_height_slider_->setValue(viewer_->convertBodyHeightToSlider(viewer_->params()->default_body_height_));
}

void HexapodParameterGui::handleExportButton()
{
  auto* params = viewer_->params();
  if (params->saveToFile("hex_config.xml"))
  {
    QMessageBox param_error;
    param_error.setWindowTitle("Success");
    param_error.setText("Parameters saved successfully.");
    param_error.exec();
  }
  else
  {
    QMessageBox param_error;
    param_error.setWindowTitle("Unable to save parameters");
    param_error.setText("Problem saving parameters!");
    param_error.exec();
  }
}

void HexapodParameterGui::handleLogDirButton()
{
  // Copy this so we can modify it below
  hebi::HexapodParameters params = *viewer_->params();

  // Get updated directory
  QString file_name = QFileDialog::getExistingDirectory(nullptr,
    tr("Select Log Directory"), params.log_path_.c_str(),
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if (file_name.size() == 0)
  {
    QMessageBox param_error;
    param_error.setWindowTitle("No folder selected");
    param_error.setText("No folder selected!");
    param_error.exec();
  }
  else
  {
    params.log_path_ = file_name.toStdString();
    viewer_->paramsUpdated(params);
  }
}
