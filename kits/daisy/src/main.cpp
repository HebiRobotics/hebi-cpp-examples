#include <QGuiApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMessageBox>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QHBoxLayout>

#include "display/hexapod_view_2d.hpp"

#include "robot/hexapod.hpp"
#include "input/input_manager_mobile_io.hpp"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <set>

using namespace hebi;
using namespace Eigen;

bool parse_parameters(int argc, char** argv, bool& visualize, bool& dummy, bool& partial, std::set<int>& partial_legs)
{
  visualize = false;
  dummy = false;
  partial = false;
  bool valid = true;
  int idx = 1; // Ignore program name!
  for (;idx < argc; ++idx)
  {
    std::string str_arg(argv[idx]);
    if (str_arg == "-h")
    {
      std::cout << "Hexapod control usage:" << std::endl <<
      "    -d" << std::endl <<
      "        Dummy hexapod; no modules are connected to.  Incompatible with \"-p\"." << std::endl << std::endl <<
      "    -p <list of integer leg indices>" << std::endl <<
      "        Creates a partial dummy hexapod; only the legs with the specified indices" << std::endl <<
      "        are connected to." << std::endl << std::endl <<
      "    -v" << std::endl <<
      "        Visualize -- show a simple rendering of the robot." << std::endl << std::endl <<
      "    -h" << std::endl <<
      "        Print this help and return." << std::endl;
      return false;
    }
    else if (str_arg == "-v")
    {
      visualize = true;
      continue;
    }
    else if (str_arg == "-d")
    {
      dummy = true;
      continue;
    }
    else if (str_arg == "-p" && idx + 1 < argc)
    {
      partial = true;
      for (++idx; idx < argc; ++idx)
      {
        try
        {
          int partial_leg = std::stoi(std::string(argv[idx]));
          partial_legs.insert(partial_leg);
        }
        catch (const std::invalid_argument&)
        {
          idx -= 2; // -2, because we will add one next start of the parent loop
          break;
        }
      }
      continue;
    }
    else
    {
      valid = false;
      break;
    }
  }
  // Do all exclusive argument checks here
  if (!valid ||
      (dummy && partial) ||
      (partial && partial_legs.size() == 0))
  {
    std::cout << "Invalid combination of arguments! Use \"-h\" for usage." << std::endl;
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  // Construct initially so I can pop up error dialogs!
  QApplication app(argc, argv);

  HexapodParameters params;
  if (!params.loadFromFile("hex_config.xml"))
  {
    params.resetToDefaults();
    QMessageBox param_error;
    param_error.setWindowTitle("Could not find parameters");
    param_error.setText("Could not find hexapod parameters, or file corrupt! Using default parameters.");
    param_error.exec();
  }

  bool do_visualize{};
  bool is_dummy{};
  HexapodErrors hex_errors;
  hex_errors.has_valid_initial_feedback = true;
  hex_errors.first_out_of_range_leg = -1;
  std::unique_ptr<Hexapod> hexapod;
  {
    bool is_partial{};
    std::set<int> legs;
    if (!parse_parameters(argc, argv, do_visualize, is_dummy, is_partial, legs))
      return -1;
    if (is_dummy)
    {
      std::cout << "Creating dummy hexapod for testing" << std::endl;
      hexapod = Hexapod::createDummy(params);
    }
    else if (is_partial)
    {
      std::cout << "Creating partial hexapod for testing" << std::endl;
      hexapod = Hexapod::createPartial(params, legs, hex_errors);
    }
    else
    {
      std::cout << "Searching network for modules..." << std::endl;
      hexapod = Hexapod::create(params, hex_errors);
    }
    if (!is_dummy)
    {
      // Try (twice) to set gains; short circuits after first success
      if (hexapod && !hexapod->setGains() && !hexapod->setGains())
      {
        QMessageBox param_error;
        param_error.setWindowTitle("Could not set gains!");
        param_error.setText("Could not set controller gains on connected modules -- this could indicate an intermittent network connection with the modules.");
        param_error.exec();
      }
      if (hexapod)
      {
        hexapod->startLogging();
      }
    }
  }

  if(!hexapod)
  {
    QMessageBox param_error;
    param_error.setWindowTitle("Could not start up hexapod");
    param_error.setText("Could not find hexapod on network -- check that all modules are visible via the Scope GUI.");
    param_error.exec();
    return -1;
  }

  if(!hex_errors.has_valid_initial_feedback)
  {
    QMessageBox param_error;
    param_error.setWindowTitle("Did not get initial feedback from modules!");
    param_error.setText("Could not get feedback from all modules -- this could indicate an intermittent network connection with the modules.");
    param_error.exec();
    return -1;
  }

  if(hex_errors.first_out_of_range_leg != -1)
  {
    QMessageBox param_error;
    param_error.setWindowTitle("Base module out of range!");
    std::string message = "Base module out of range (leg " + std::to_string(hex_errors.first_out_of_range_leg + 1) + ")";
    param_error.setText(QString(message.c_str()));
    param_error.exec();
    return -1;
  }

  std::unique_ptr<input::InputManager> input(new input::InputManagerMobileIO());
  // TODO: retry a "reset" this multiple times? Wait in a for loop?
  if (!input->isConnected())
  {
    std::cout << "Could not find I/O board for joystick." << std::endl;
    return -1;
  }


  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Found robot -- starting control program." << std::endl;

  double overall_width = do_visualize ? 1100 : 300;
  double overall_height = do_visualize ? 800 : 150;

  QWidget *widget = new QWidget;

  // These could be put into a struct -- basically, we just need to hold on to
  // these objects during application runtime, and they may be optional. Should
  // really refactor more completely here.
  QLabel* mode;
  QLabel* feedback_status;
  std::unique_ptr<QGraphicsScene> scene;
  std::unique_ptr<HexapodView2D> hexapod_display;
  std::unique_ptr<QGraphicsView> view;

  if (do_visualize)
  {
    scene.reset(new QGraphicsScene(QRectF(0, 0, overall_width - 200, overall_height)));
    hexapod_display.reset(new HexapodView2D(scene.get()));

    view.reset(new QGraphicsView(scene.get()));
    view->setBackgroundBrush(Qt::black);
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->resize(overall_width + 10, overall_height + 10);

    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setAlignment(Qt::AlignTop);
    hLayout->addWidget(view.get(), 1);
    hLayout->addLayout(vLayout);

    ////////////////////////
    // Build the panel with buttons/labels/etc.
    // TODO: abstract out to a different class?
    QLabel* mode_label = new QLabel("Current Mode:");
    auto font = mode_label->font();
    font.setWeight(QFont::Bold);
    mode_label->setFont(font);
    mode = new QLabel("---");

    QLabel* feedback_status_label = new QLabel("Module Ethernet Connection:");
    font = feedback_status_label->font();
    font.setWeight(QFont::Bold);
    feedback_status_label->setFont(font);

    feedback_status = new QLabel("Good");
    QPalette palette = feedback_status->palette();
    palette.setColor(QPalette::Background, Qt::green);
    feedback_status->setAutoFillBackground(true);
    feedback_status->setPalette(palette);

    QLabel* frequency_label = new QLabel("Logging Frequencies (low/high):");
    font = frequency_label->font();
    font.setWeight(QFont::Bold);
    frequency_label->setFont(font);
    std::string frequency_text = std::to_string(params.low_log_frequency_hz_) + "/" + std::to_string(params.high_log_frequency_hz_);
    QLabel* frequency = new QLabel(frequency_text.c_str());

    vLayout->addWidget(mode_label);
    vLayout->addWidget(mode);
    vLayout->addWidget(feedback_status_label);
    vLayout->addWidget(feedback_status);
    vLayout->addWidget(frequency_label);
    vLayout->addWidget(frequency);
    ////////////////////////
  }
  else
  {
    QVBoxLayout *vLayout = new QVBoxLayout(widget);
    QHBoxLayout *hLayout1 = new QHBoxLayout();
    QHBoxLayout *hLayout2 = new QHBoxLayout();
    QHBoxLayout *hLayout3 = new QHBoxLayout();
    QLabel* mode_label = new QLabel("Current Mode:");
    auto font = mode_label->font();
    font.setWeight(QFont::Bold);
    mode_label->setFont(font);
    mode = new QLabel("---");

    QLabel* feedback_status_label = new QLabel("Module Ethernet Connection:");
    font = feedback_status_label->font();
    font.setWeight(QFont::Bold);
    feedback_status_label->setFont(font);
    feedback_status = new QLabel("Good");
    QPalette palette = feedback_status->palette();
    palette.setColor(QPalette::Background, Qt::green);
    feedback_status->setAutoFillBackground(true);
    feedback_status->setPalette(palette);

    QLabel* frequency_label = new QLabel("Logging Frequencies (low/high):");
    font = frequency_label->font();
    font.setWeight(QFont::Bold);
    frequency_label->setFont(font);
    std::string frequency_text = std::to_string(params.low_log_frequency_hz_) + "/" + std::to_string(params.high_log_frequency_hz_);
    QLabel* frequency = new QLabel(frequency_text.c_str());

    hLayout1->addWidget(mode_label);
    hLayout1->addWidget(mode);
    hLayout2->addWidget(feedback_status_label);
    hLayout2->addWidget(feedback_status);
    hLayout3->addWidget(frequency_label);
    hLayout3->addWidget(frequency);
    vLayout->addLayout(hLayout1);
    vLayout->addLayout(hLayout2);
    vLayout->addLayout(hLayout3);
  }

  widget->setWindowTitle(QStringLiteral("heXapod Control"));
  widget->show();
  widget->resize(overall_width, overall_height);

  Eigen::Vector3f translation_velocity_cmd;
  translation_velocity_cmd << 0, 0, 0;
  Eigen::Vector3f rotation_velocity_cmd;
  rotation_velocity_cmd << 0, 0, 0;

//  Eigen::Matrix3f orientation;
  double period = 5.0; // in milliseconds; e.g., 5 ms => 1000/5 = 200 Hz

  // TODO: consider adding rendering of controls here.
/* 
  draw_controls.draw([&](paint::graphics& graph)
  {
    int vx = (int)(translation_velocity_cmd(0) * 200.0);
    int vy = (int)(translation_velocity_cmd(1) * 200.0);
    int vz = (int)(translation_velocity_cmd(2) * 200.0);
    //int rx = (int)(rotation_velocity_cmd(0) * 200.0);
    //int ry = (int)(rotation_velocity_cmd(1) * 200.0);
    int rz = (int)(rotation_velocity_cmd(2) * 200.0);
    int start_x = controls_panel.size().width / 2;
    int start_y = 100;
    // z/y vel
    graph.line(point{start_x, start_y}, point{start_x + vy, start_y + vx}, colors::black);
    // rotation
    graph.rectangle(rectangle{(int)(start_x + std::min(rz, 0)), start_y, (unsigned)std::abs(rz), 20}, true, colors::blue );
    // z height vel
    graph.rectangle(rectangle{start_x, (int)(start_y + std::min(vz, 0)), 20, (unsigned)std::abs(vz)}, true, colors::red );
  });

  draw_controls.update();
*/
  bool startup = true;
  bool first_run = true;
  bool last_connection_status = true;
  bool high_freq_logging = false;
  double startup_seconds = 4.5;

  // Controls to send to the robot
  Eigen::VectorXd angles(Leg::getNumJoints());
  Eigen::VectorXd vels(Leg::getNumJoints());
  Eigen::VectorXd torques(Leg::getNumJoints());
  Eigen::Vector3d gravity_vec;
  gravity_vec << 0, 0, -9.8;
  Eigen::MatrixXd foot_forces(3,6); // 3 (xyz) by num legs
  std::vector<std::shared_ptr<trajectory::Trajectory>> startup_trajectories;

  auto start = std::chrono::steady_clock::now();
  long interval_ms = period;
  // http://stackoverflow.com/questions/30425772/c-11-calling-a-c-function-periodically
  std::atomic<bool> control_execute;
  control_execute.store(true, std::memory_order_release);
  std::thread control_thread([&]()
  {
    auto prev = std::chrono::steady_clock::now();
    // Get dt (in seconds)
    std::chrono::duration<double> dt = std::chrono::seconds(0);
    while (control_execute.load(std::memory_order_acquire))
    {

    // Wait!
    auto now = std::chrono::steady_clock::now();
    auto need_to_wait = std::max(0, (int)std::chrono::duration_cast<std::chrono::milliseconds>(prev + std::chrono::milliseconds(interval_ms) - now).count());
    std::this_thread::sleep_for(std::chrono::milliseconds(need_to_wait));

    // Get dt (in seconds)
    now = std::chrono::steady_clock::now();
    dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
    prev = now;

    if (!is_dummy)
    {
      // More than 2x the feedback period?
      if ((now - hexapod->getLastFeedbackTime()) > std::chrono::milliseconds((long)(hexapod->getFeedbackPeriodMs() * 2.0)))
      {
        if (last_connection_status)
        {
          feedback_status->setText("Intermittent");
          QPalette palette = feedback_status->palette();
          palette.setColor(QPalette::Background, Qt::red);
          feedback_status->setPalette(palette);
        }
        last_connection_status = false;
      }
      else
      {
        if (!last_connection_status)
        {
          feedback_status->setText("Good");
          QPalette palette = feedback_status->palette();
          palette.setColor(QPalette::Background, Qt::green);
          feedback_status->setPalette(palette);
        }
        last_connection_status = true;
      }
    }

    // In seconds:
    std::chrono::duration<double> elapsed(now - start);

    // Get joystick update, and update any relevant variables.
    input->update();
    if (input->getQuitButtonPushed())
      app.exit();

    translation_velocity_cmd = input->getTranslationVelocityCmd();
    rotation_velocity_cmd = input->getRotationVelocityCmd();
    hexapod->updateMode(input->getAndResetModeToggleCount());

    // The first minute of every 30 minutes:
    if (fmod(elapsed.count(), 1800) < 60)
    {
      if (!high_freq_logging)
      {
        high_freq_logging = true;
        if (hexapod->hasLogGroup())
          hexapod->setLoggingFrequency(params.high_log_frequency_hz_);
      }
    }
    else
    {
      if (high_freq_logging)
      {
        high_freq_logging = false;
        if (hexapod->hasLogGroup())
          hexapod->setLoggingFrequency(params.low_log_frequency_hz_);
      }
    }
//    hexapod->getOrientation(orientation);

    // Startup phase: smoothly transition!
    if (startup)
    {
      int num_joints = Leg::getNumJoints();
      // TODO: move all logic inside leg?
      if (first_run)
      {
        // V l in legs, let s_l = start joints and e_l = end joints and t_l =
        // trajectory, with zero vel/accel endpoints.
        for (int i = 0; i < 6; ++i)
        {
          Eigen::VectorXd leg_start = hexapod->getLegFeedback(i);
          Eigen::VectorXd leg_end;
          Eigen::VectorXd leg_vels;
          Eigen::MatrixXd jacobian_ee(0, 0);
          robot_model::MatrixXdVector jacobian_com;
          hexapod->getLeg(i)->computeState(elapsed.count(), leg_end, leg_vels, jacobian_ee, jacobian_com);
          // TODO: fix! (quick and dirty -- leg mid is hardcoded as offset from leg end)
          Eigen::VectorXd leg_mid = leg_end;
          leg_mid(1) -= 0.3;
          leg_mid(2) -= 0.15;

          // Convert for trajectories
          int num_waypoints = 5;
          Eigen::MatrixXd positions(num_joints, num_waypoints);
          Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
          Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
          Eigen::VectorXd nan_column = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
          // Is this one of the legs that takes a step first?
          bool step_first = (i == 0 || i == 3 || i == 4);

          // Set positions
          positions.col(0) = leg_start;
          positions.col(1) = step_first ? leg_mid : leg_start;
          positions.col(2) = step_first ? leg_end : leg_start;
          positions.col(3) = step_first ? leg_end : leg_mid;
          positions.col(4) = leg_end;

          velocities.col(1) = nan_column;
          velocities.col(3) = nan_column;
          accelerations.col(1) = nan_column;
          accelerations.col(3) = nan_column;

          Eigen::VectorXd times(num_waypoints);
          double local_start = elapsed.count();
          double total = startup_seconds - local_start;
          times << local_start,
                   local_start + total * 0.25,
                   local_start + total * 0.5,
                   local_start + total * 0.75,
                   local_start + total;
          startup_trajectories.push_back(trajectory::Trajectory::createUnconstrainedQp(
            times, positions, &velocities, &accelerations));

        }

        // TODO: move this startup logic outside of t.elapse() call, and then
        // we don't need 'first_run'?
        first_run = false;
      }
      mode->setText("Startup");
     
      // Follow t_l:
      // TODO
      for (int i = 0; i < 6; ++i)
      {
        Eigen::VectorXd v(3);
        Eigen::VectorXd a(3);
        startup_trajectories[i]->getState(elapsed.count(), &angles, &v, &a);
        vels = v;
        // TODO: FILL IN JACOBIAN DURING STARTUP!
        Eigen::MatrixXd jacobian_ee(0,0);
        robot_model::MatrixXdVector jacobian_com;
        Eigen::Vector3d foot_force = foot_forces.block<3,1>(0,i);
        hebi::Leg* curr_leg = hexapod->getLeg(i);
        torques = curr_leg->computeTorques(jacobian_com, jacobian_ee, angles, vels, gravity_vec, /* dynamic_comp_torque,*/ foot_force); // TODO:
        // For rendering:
        if (hexapod_display)
          hexapod_display->updateLeg(curr_leg, i, angles);
        // TODO: add actual foot torque for startup?
        // TODO: add vel, torque; test each one!
        hexapod->setCommand(i, &angles, &vels, &torques);
      }
      hexapod->sendCommand();
      if (elapsed.count() >= startup_seconds)
      {
        mode->setText("");
        startup = false;
      }
      continue;
    }

    mode->setText(hexapod->getMode() == hebi::Hexapod::Mode::Step ? "Step" : "Stance");

    hexapod->updateStance(
      translation_velocity_cmd.cast<double>(),
      rotation_velocity_cmd.cast<double>(),
      dt.count());

    if (hexapod->needToStep())
    {
      hexapod->startStep(elapsed.count());
    }

    hexapod->updateSteps(elapsed.count());

    // Calculate how the weight is distributed
    hexapod->computeFootForces(elapsed.count(), foot_forces);

    Eigen::MatrixXd jacobian_ee;
    robot_model::MatrixXdVector jacobian_com;
    Eigen::VectorXd angles_plus_dt;
    for (int i = 0; i < 6; ++i)
    {
      hebi::Leg* curr_leg = hexapod->getLeg(i);
      // TODO: add vels and torques
      curr_leg->computeState(elapsed.count(), angles, vels, jacobian_ee, jacobian_com);

      // For rendering:
      if (hexapod_display)
        hexapod_display->updateLeg(curr_leg, i, angles);
      
      // Get torques
      Eigen::Vector3d foot_force = foot_forces.block<3,1>(0,i);
      torques = hexapod->getLeg(i)->computeTorques(jacobian_com, jacobian_ee, angles, vels, gravity_vec, /*dynamic_comp_torque,*/ foot_force); // TODO:

      hexapod->setCommand(i, &angles, &vels, &torques);
    }
    hexapod->sendCommand();
    }
  });
  bool res = app.exec();
  control_execute.store(false, std::memory_order_release);
  control_thread.join();
  return res;
}
