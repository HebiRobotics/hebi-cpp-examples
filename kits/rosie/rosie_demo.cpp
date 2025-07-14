// HEBI C++ API files:
#include "arm/arm.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"
#include "util/mobile_io.hpp"
#include "util/vector_utils.h"

// Common includes
#include <iostream>

using namespace hebi;

Eigen::Matrix3d makeRotationMatrix (hebi::Quaternionf phone_orientation) {
  Eigen::Quaterniond q;
  q.w() = phone_orientation.getW();
  q.x() = phone_orientation.getX();
  q.y() = phone_orientation.getY();
  q.z() = phone_orientation.getZ();
  return q.toRotationMatrix();
}

struct OmniBase {
  // Create omnibase and initialize the command
  OmniBase(hebi::Group& group) : group_(group), cmd_(group.size()) {
    GroupFeedback wheel_fbk(group_.size());
    while (!group_.getNextFeedback(wheel_fbk))
      std::cout << "Couldn't get feedback from the wheels!\n";
    cmd_.setPosition(wheel_fbk.getPosition());
    last_time_ = wheel_fbk.getTime();
    traj_start_time_ = last_time_;
    
    // Initialize base trajectory
    omni_base_traj_time_ << 0, base_ramp_time;
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(3,2); 
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(3,2); 
    Eigen::MatrixXd jerks = Eigen::MatrixXd::Zero(3,2); 
    traj_ = trajectory::Trajectory::createUnconstrainedQp(omni_base_traj_time_, velocities, &accelerations, &jerks);
  }

  bool setGains()
  {
    GroupCommand base_gains_command(group_.size());
    if (!base_gains_command.readGains("gains/omni-drive-wheel-gains.xml")) {
      std::cout << "Could not read omni base gains\n";
      return false;
    }
    if (!group_.sendCommandWithAcknowledgement(base_gains_command)) {
      std::cout << "Could not send omni base gains\n";
      return false;
    }
    return true;
  }

  // Evaluate Trajectory State and update commands
  void update(double t, double x_vel, double y_vel, double rot_vel)
  {
    double dt = t - last_time_;
    last_time_ = t;

    double traj_time = std::min(traj_->getDuration(), t - traj_start_time_);
    Eigen::VectorXd chassis_cmd_vel(base_num_wheels_);
    Eigen::VectorXd chassis_cmd_acc(base_num_wheels_);
    Eigen::VectorXd chassis_cmd_jerk(base_num_wheels_);
    traj_->getState(traj_time, &chassis_cmd_vel, &chassis_cmd_acc, &chassis_cmd_jerk);
    
    // Build commands from trajectory
    cmd_.setVelocity(base_wheel_velocity_matrix_ * chassis_cmd_vel);
    cmd_.setPosition(cmd_.getPosition() + cmd_.getVelocity() * dt);
    cmd_.setEffort(base_wheel_effort_matrix_ * (chassis_mass_matrix_ * chassis_cmd_acc));

    // Rebuild trajectory
    Eigen::Vector3d chassis_desired_vel;
    chassis_desired_vel << base_max_lin_speed_ * x_vel, -base_max_lin_speed_ * y_vel, base_max_rot_speed_ * rot_vel;
      
    Eigen::MatrixXd velocities(base_num_wheels_, 2);
    velocities.col(0) = chassis_cmd_vel;
    velocities.col(1) = chassis_desired_vel;
      
    Eigen::MatrixXd accelerations(base_num_wheels_, 2);
    accelerations.col(0) = chassis_cmd_acc;
    accelerations.col(1) = Eigen::Vector3d::Zero();
  
    Eigen::MatrixXd jerks(base_num_wheels_, 2);
    jerks.col(0) = chassis_cmd_jerk;
    jerks.col(1) = Eigen::Vector3d::Zero();

    traj_ = hebi::trajectory::Trajectory::createUnconstrainedQp(omni_base_traj_time_, velocities, &accelerations , &jerks);
    traj_start_time_ = t;
  }

  void send()
  {
    group_.sendCommand(cmd_);
  }

private:
  double last_time_{};
  double traj_start_time_{};
  const int base_num_wheels_{3};
  hebi::Group& group_;
  hebi::GroupCommand cmd_;
  std::shared_ptr<trajectory::Trajectory> traj_;

  const double a1 = -60*M_PI/180;
  const double a2 = 60*M_PI/180;
  const double a3 = 180*M_PI/180;

  const double base_wheel_base_ = 0.470;
  const double base_wheel_radius_ = .15/2.0;
  
  const double base_chassis_mass_ = 12.0;
  const double base_chassis_inertia_zz_ = .5*base_chassis_mass_ * base_wheel_base_ * base_wheel_base_ *.25;
  const Eigen::MatrixXd chassis_mass_matrix_
    {(Eigen::MatrixXd(3,3) << base_chassis_mass_,0,0, 0,base_chassis_mass_,0, 0,0,base_chassis_inertia_zz_).finished()};

  const Eigen::Matrix3d base_wheel_transform_
    {(Eigen::MatrixXd(3,3) <<
     sin(a1), -cos(a1), 2/base_wheel_base_,
     sin(a2), -cos(a2), 2/base_wheel_base_/2,
     sin(a3), -cos(a3), 2/base_wheel_base_/2).finished()};
  const Eigen::Matrix3d base_wheel_velocity_matrix_{base_wheel_transform_ / base_wheel_radius_};
  const Eigen::Matrix3d base_wheel_effort_matrix_{base_wheel_transform_ * base_wheel_radius_};

  const double base_ramp_time = .33;

  const double base_max_lin_speed_ = 0.6;
  const double base_max_rot_speed_ = base_max_lin_speed_*(base_wheel_base_/2.0);

  Eigen::Vector2d omni_base_traj_time_;
};

int main() {

  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  // Note -- this is currently for an R-series Rosie
  const std::string example_config_file = "rosie-r.cfg.yaml";
  std::string example_config_path;
  std::vector<std::string> errors;

  if (!example_config_file.empty()) {
    example_config_path = "config/" + example_config_file;
	printf("Using config file: %s\n", example_config_path.c_str());
  }
  else {
    example_config_path = "config/rosie-r.cfg.yaml";
  }

  // Load the config
  const auto example_config = RobotConfig::loadConfig(example_config_path, errors);
  for (const auto& error : errors) {
    std::cerr << error << std::endl;
  }
  if (!example_config) {
    std::cerr << "Failed to load configuration from: " << example_config_path << std::endl;
    return -1;
  }

  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Create the arm object from the configuration, and keep retrying if arm not found
  auto arm = arm::Arm::create(*example_config);
  while (!arm) {
    std::cerr << "Failed to create arm, retrying..." << std::endl;
    arm = arm::Arm::create(*example_config);
    return 1;
  }
  std::cout << "Arm connected." << std::endl;

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<util::MobileIO> mobile_io = util::MobileIO::create("HEBI", "mobileIO");
  while (!mobile_io)
  {
    std::cout << "couldn't find mobile IO device!\n";
  }

  // Clear any garbage on screen
  mobile_io->resetUI();

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1: Home  B2: Track\n"
                  "A6: Grip  B8: Quit\n"
                  "A3: XYZ   B3: Compliant\n");

  // Display instructions on screen
  mobile_io->appendText(instructions);

  mobile_io->setButtonLabel(1, "Home");
  mobile_io->setButtonLabel(2, "Track");
  mobile_io->setButtonLabel(8, "Quit");
  mobile_io->setAxisLabel(2, "Base Turn");
  mobile_io->setAxisLabel(3, "Arm Translate");
  mobile_io->setAxisLabel(6, "Grip");
  mobile_io->setAxisLabel(7, "Base X");
  mobile_io->setAxisLabel(8, "Base Y");

  // Get initial mobile device state (so edge triggers are properly handled
  // after this)
  mobile_io->update();

  std::cout << instructions << "\n";

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  const auto user_data = example_config->getUserData();
  const Eigen::VectorXd ik_seed_position = util::stdToEigenXd(user_data.getFloatList("ik_seed_pos"));
  const double soft_start_time = user_data.getFloat("homing_duration");
  const Eigen::VectorXd xyz_scale = util::stdToEigenXd(user_data.getFloatList("xyz_scale"));
  const double delay_time = user_data.getFloat("delay_time");
  const double gripper_open_effort = user_data.getFloat("gripper_open_effort");
  const double gripper_close_effort = user_data.getFloat("gripper_close_effort");

  // Different modes it can be in (when in none, then automatic grav_comp)
  //bool softstart = true;
  bool ar_mode = false;
  
  // Set up states for the mobile device.  Note -- coupled to ar_mode above, and only used 
  // when that is "true".
  Eigen::Vector3d xyz_phone_init;
  Eigen::Matrix3d rot_phone_init;
 
  bool enable_logging = true;

  std::string robot_family = "Rosie";
  
  ////////////////////
  // Get groups and set the Gains
  ////////////////////
     
  Lookup lookup;
  // Optional step to limit the lookup to a set of interfaces or modules
  // lookup.setLookupAddresses('10.10.10.255');
  const std::vector<std::string> base_wheel_module_names = {"W1","W2","W3"};
  auto wheel_group = lookup.getGroupFromNames({robot_family}, base_wheel_module_names);
  if (!wheel_group || wheel_group->size() != 3)
  {
    std::cout << "Could not find wheel modules on network!\n";
    return 1;
  }

  // Setup for base trajectory
  OmniBase base(*wheel_group);
  if (!base.setGains())
    return 1;

  auto gripper = hebi::arm::Gripper::create(robot_family, "gripperSpool", gripper_close_effort, gripper_open_effort); 
  std::string gripper_gains_file = example_config->getGains("gripper");
  printf("Gripper gains file: %s\n", gripper_gains_file.c_str());
  gripper_gains_file = "home/hebi/KHEBI/hebi-cpp-examples/build/kits/arms/config/gains/A-2255-01.xml";
  if (!gripper || !gripper->loadGains(example_config_path + "//" + gripper_gains_file))
  {
	std::cout << "Could not load config file: " << example_config_path << "\n";
    std::cout << "Could not load gains file: " << gripper_gains_file << "\n";
    std::cout << "Could not read or send gripper gains\n";
  }

  gripper_gains_file = "../arms/config/gains/A-2255-01.xml";
  if (!gripper || !gripper->loadGains(example_config_path + "//" + gripper_gains_file))
  {
      std::cout << "Could not load config file: " << example_config_path << "\n";
      std::cout << "Could not load gains file: " << gripper_gains_file << "\n";
      std::cout << "Could not read or send gripper gains\n";
  }

  return 1;
  // Start background logging
  if(enable_logging) {
    arm->group().startLog("logs");
    wheel_group->startLog("logs");
  }

  ////////////////////
  // Begin the demo loop
  ////////////////////

  std::cout << "Starting demo...\n";
  
  // Home position
  Eigen::Vector3d xyz_home;
  xyz_home << 0.3, 0.0, 0.3;
  // Gripper down
  Eigen::Matrix3d rot_home(3,3);
  rot_home << -1,0,0, 0,1,0, 0,0,-1; 

  // Note -- could just hardcode here, too...
  // Calculate new arm joint angle targets
  auto target_joints = arm->solveIK(ik_seed_position,
                                    xyz_home, 
                                    rot_home);
  arm->update();
  arm->setGoal(arm::Goal::createFromPosition(soft_start_time, target_joints));
  bool soft_start = true;
  
  while (!arm->atGoal())
  {
    arm->update();
    arm->send();
  }

  // Omnibase commands:
  double x_vel{}; 
  double y_vel{};
  double rot_vel{};

  int num_mobile_io_drops = 0;

  // Only allow rotation from AR
  bool rot_only{false};
  Eigen::Vector3d last_xyz_phone;

  Eigen::Vector3d xyz_target;

  while(true) {
    arm->update();

    // Get latest mobile_state (use timeout of zero)
    auto updated_mobile_io = mobile_io->update(0);
    if (!updated_mobile_io) {
      ++num_mobile_io_drops;
      // 10 in a row? ~ 0.1s?  stop commanding velocities...
      if (num_mobile_io_drops > 10)
      {
        x_vel = 0;
        y_vel = 0;
        rot_vel = 0;
      }
    } else {
      num_mobile_io_drops = 0;
      // Button B1 - Return to home position
      if (mobile_io->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
        ar_mode = false;

        // Note: could use "home_position" instead...
        auto target_joints = arm->solveIK(ik_seed_position,
                                          xyz_home, 
                                          rot_home);
        arm->setGoal(arm::Goal::createFromPosition(soft_start_time, target_joints));
      }

      // Button B2 - Toggle AR Control
      if (mobile_io->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn) {
        if (!ar_mode) { // -> AR mode
          xyz_phone_init << mobile_io->getLastFeedback().mobile().arPosition().get().getX(),
                            mobile_io->getLastFeedback().mobile().arPosition().get().getY(),
                            mobile_io->getLastFeedback().mobile().arPosition().get().getZ();
          rot_phone_init = makeRotationMatrix(mobile_io->getLastFeedback().mobile().arOrientation().get());
          ar_mode = true;
        }
      }
      // -> grav comp
      if (mobile_io->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
        if (ar_mode) {
          arm->cancelGoal();
          ar_mode = false;
        }
      }

      // Button B8 - End Demo
      if (mobile_io->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile_io->resetUI();
        return 1;
      }

      if (mobile_io->getAxis(3) < 0) {
        if (!rot_only)
        {
          rot_only = true;
          last_xyz_phone << mobile_io->getLastFeedback().mobile().arPosition().get().getX(),
                            mobile_io->getLastFeedback().mobile().arPosition().get().getY(),
                            mobile_io->getLastFeedback().mobile().arPosition().get().getZ();
        }
      } else {
        if (rot_only) // transition out
        {
          rot_only = false;
          Eigen::Vector3d xyz_phone;
          xyz_phone << mobile_io->getLastFeedback().mobile().arPosition().get().getX(),
                       mobile_io->getLastFeedback().mobile().arPosition().get().getY(),
                       mobile_io->getLastFeedback().mobile().arPosition().get().getZ();
          xyz_phone_init += xyz_phone - last_xyz_phone;
        }
      }

      // Gripper Control
      // (for states, 0 is open, 1 is closed)
      gripper->setState((mobile_io->getAxis(6) + 1.0 ) / 2.0); 

      // Omnibase
      x_vel = mobile_io->getAxis(8);
      y_vel = mobile_io->getAxis(7);
      rot_vel = mobile_io->getAxis(1);
    }

    if (ar_mode) {
      // Get the latest mobile position and orientation
      Eigen::Vector3d xyz_phone;
      xyz_phone << mobile_io->getLastFeedback().mobile().arPosition().get().getX(),
                  mobile_io->getLastFeedback().mobile().arPosition().get().getY(),
                  mobile_io->getLastFeedback().mobile().arPosition().get().getZ();
      auto rot_phone = makeRotationMatrix(mobile_io->getLastFeedback().mobile().arOrientation().get());

      // Calculate new targets
      if (!rot_only)
      {
        xyz_target =
          xyz_home + xyz_scale.cwiseProduct(rot_phone_init.transpose() * (xyz_phone - xyz_phone_init));
      }
      Eigen::Matrix3d rot_target = rot_phone_init.transpose() * rot_phone * rot_home;

      // Force elbow up config
      auto seed_pos_ik = arm->lastFeedback().getPosition();
      seed_pos_ik[2] = abs(seed_pos_ik[2]);

      // Calculate new arm joint angle targets
      target_joints = arm->solveIK(seed_pos_ik, xyz_target, rot_target);

      // Create and send new goal to the arm
      arm->setGoal(arm::Goal::createFromPosition(delay_time, target_joints));
    }

    // Send latest commands to the arm
    arm->send();
    gripper->send();

    // Base control
    base.update(arm->lastFeedback().getTime(), x_vel, y_vel, rot_vel);
    base.send();
  }
}
