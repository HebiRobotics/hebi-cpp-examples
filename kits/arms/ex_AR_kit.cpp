/*
 * AR Kit Example
 * Currently assumes 6 degrees of freedom in you arm!
 * This demo sets up your arm's end effector to mimick the changes in 
 * position and orientation of your mobile IO device.
 */

// HEBI C++ API files:
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
// Local example utils:
#include "util/vector_utils.h"
// C++ standard libraries:
#include <chrono>
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


int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/ex_AR_kit.cfg.yaml";
  std::vector<std::string> errors;
  
  // Load the config
  const auto example_config = RobotConfig::loadConfig(example_config_file, errors);
  for (const auto& error : errors) {
    std::cerr << error << std::endl;
  }
  if (!example_config) {
    std::cerr << "Failed to load configuration from: " << example_config_file << std::endl;
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
  }
  std::cout << "Arm connected." << std::endl;


  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<util::MobileIO> mobile_io = util::MobileIO::create("Arm", "mobileIO");
  if (!mobile_io)
  {
    std::cout << "couldn't find mobile IO device!\n";
    return 1;
  }

  // Clear any garbage on screen
  mobile_io->resetUI();

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Home Position\nB3 - AR Control Mode\n"
                  "B6 - Grav Comp Mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile_io->appendText(instructions);
  // Get initial mobile device state (so edge triggers are properly handled
  // after this)
  mobile_io->update();

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Different modes it can be in (when in none, then automatic grav_comp)
  bool softstart = true;
  bool ar_mode = false;
  
  // Load user data from config
  const auto user_data = example_config->getUserData();
  Eigen::VectorXd home_position = util::stdToEigenXd(user_data.getFloatList("home_position"));
  double soft_start_time = user_data.getFloat("homing_duration");
  Eigen::VectorXd xyz_scale = util::stdToEigenXd(user_data.getFloatList("xyz_scale"));
  double delay_time = user_data.getFloat("delay_time");

  // Command the softstart to home position
  arm -> update();
  arm -> setGoal(arm::Goal::createFromPosition(soft_start_time, home_position)); // move to goal over "soft_start_time" seconds
  arm -> send();

  // Get the cartesian position and rotation matrix @ home position
  Eigen::Vector3d xyz_home; 
  Eigen::Matrix3d rot_home;
  arm -> FK(home_position, xyz_home, rot_home); 

  // Set up states for the mobile device
  Eigen::Vector3d xyz_phone_init;
  Eigen::Matrix3d rot_phone_init;

  // Target variables
  Eigen::VectorXd target_joints(arm -> robotModel().getDoFCount());

  while(arm->update())
  {

    if (softstart) {
      // End softstart when arm reaches its homePosition
      if (arm -> atGoal()) {
        mobile_io->appendText("Softstart Complete!");
        softstart = false; 
        continue;
      }
      arm -> send();

      // Stay in softstart, don't do any other behavior
      continue;
    }

    // Get latest mobile_state
    auto updated_mobile_io = mobile_io->update(0);
      
    if (!updated_mobile_io)
      std::cout << "Failed to get feedback from mobile I/O; check connection!\n";
    else
    {
      // Button B1 - Return to home position
      if (mobile_io->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
        ar_mode = false;
        arm -> setGoal(arm::Goal::createFromPosition(soft_start_time, home_position));
      }

      // Button B3 - Start AR Control
      if (mobile_io->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
        xyz_phone_init << mobile_io -> getLastFeedback().mobile().arPosition().get().getX(),
                          mobile_io -> getLastFeedback().mobile().arPosition().get().getY(),
                          mobile_io -> getLastFeedback().mobile().arPosition().get().getZ();
        rot_phone_init = makeRotationMatrix(mobile_io -> getLastFeedback().mobile().arOrientation().get());
        ar_mode = true;
      }

      // Button B6 - Grav Comp Mode
      if (mobile_io->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn) {
        arm -> cancelGoal();
        ar_mode = false;
      }

      // Button B8 - End Demo
      if (mobile_io->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile_io->resetUI();
        return 1;
      }
    }

    if (ar_mode) {
      // Get the latest mobile position and orientation
      Eigen::Vector3d xyz_phone;
      xyz_phone << mobile_io->getLastFeedback().mobile().arPosition().get().getX(),
                   mobile_io->getLastFeedback().mobile().arPosition().get().getY(),
                   mobile_io->getLastFeedback().mobile().arPosition().get().getZ();
      auto rot_phone = makeRotationMatrix(mobile_io->getLastFeedback().mobile().arOrientation().get());

      // Calculate new targets
      Eigen::Vector3d xyz_target =
        xyz_home + xyz_scale.cwiseProduct(rot_phone_init.transpose() * (xyz_phone - xyz_phone_init));
      Eigen::Matrix3d rot_target = rot_phone_init.transpose() * rot_phone * rot_home;

      // Calculate new arm joint angle targets
      target_joints = arm -> solveIK(arm -> lastFeedback().getPosition(),
                                              xyz_target, 
                                              rot_target);

      // Create and send new goal to the arm
      arm -> setGoal(arm::Goal::createFromPosition(delay_time, target_joints));
    }

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile_io->resetUI();

  return 0;
}
