/*
 * AR Kit Example
 * Currently assumes 6 degrees of freedom in you arm!
 * This demo sets up your arm's end effector to mimick the changes in 
 * position and orientation of your mobile IO device.
 */

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>
#include <iostream>

using namespace hebi;
using namespace experimental; // For all things mobileIO 



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


  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  std::unique_ptr<util::MobileIO> mobile = util::MobileIO::create("Arm", "mobileIO");
  if (!mobile)
  {
    std::cout << "couldn't find mobile IO device!\n";
    return 1;
  }

  // Clear any garbage on screen
  mobile->resetUI();

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Home Position\nB3 - AR Control Mode\n"
                  "B6 - Grav Comp Mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile->appendText(instructions); 

  // Setup state variable for mobile device
  auto last_mobile_state = mobile->update();

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Different modes it can be in (when in none, then automatic grav_comp)
  bool softstart = true;
  bool ar_mode = false;
  
  // Make sure we softstart the arm first.
  Eigen::VectorXd home_position(arm -> robotModel().getDoFCount());
  home_position << 0, M_PI/3, 2*M_PI/3, 5*M_PI/6, -M_PI/2, 0; // Adjust depending on your DoFs

  // Command the softstart to home position
  arm -> update();
  arm -> setGoal(arm::Goal::createFromPosition(4, home_position)); // take 4 seconds
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
      if (arm -> atGoal()){
        mobile->appendText("Softstart Complete!");
        softstart = false; 
        continue;
        }
        arm -> send();

        // Stay in softstart, don't do any other behavior
        continue;
      } 

      // Get latest mobile_state
      auto updated_mobile = mobile->update(0);
      
      if (!updated_mobile)
        std::cout << "Failed to get feedback from mobile I/O; check connection!\n";
      else
      {
      // Button B1 - Return to home position
      if (mobile->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
          ar_mode = false;
          arm -> setGoal(arm::Goal::createFromPosition(4, home_position));
      }

      // Button B3 - Start AR Control
      if (mobile->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
        xyz_phone_init << mobile -> getLastFeedback().mobile().arPosition().get().getX(),
                          mobile -> getLastFeedback().mobile().arPosition().get().getY(),
                          mobile -> getLastFeedback().mobile().arPosition().get().getZ();
        std::cout << xyz_phone_init << std::endl;
        rot_phone_init = makeRotationMatrix(mobile -> getLastFeedback().mobile().arOrientation().get());
        ar_mode = true;
      }

      // Button B6 - Grav Comp Mode
      if (mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn) {
        arm -> cancelGoal();
        ar_mode = false;
      }

      // Button B8 - End Demo
      if (mobile->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile->resetUI();
        return 1;
      }
    }

    if (ar_mode) {
      // Get the latest mobile position and orientation
      Eigen::Vector3d xyz_phone;
      xyz_phone << mobile->getLastFeedback().mobile().arPosition().get().getX(),
                   mobile->getLastFeedback().mobile().arPosition().get().getY(),
                   mobile->getLastFeedback().mobile().arPosition().get().getZ();
      auto rot_phone = makeRotationMatrix(mobile->getLastFeedback().mobile().arOrientation().get());

      // Calculate new targets
      Eigen::Vector3d xyz_scale;
      xyz_scale << 1, 1, 2;
      Eigen::Vector3d xyz_target = xyz_home + (0.75 * xyz_scale.array() *
                                (rot_phone_init.transpose() * (xyz_phone - xyz_phone_init)).array()).matrix();
      Eigen::Matrix3d rot_target = rot_phone_init.transpose() * rot_phone * rot_home;

      // Calculate new arm joint angle targets
      target_joints = arm -> solveIK(arm -> lastFeedback().getPosition(),
                                              xyz_target, 
                                              rot_target);

      // Create and send new goal to the arm
      arm -> setGoal(arm::Goal::createFromPosition(target_joints));
    }

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile->resetUI();

  return 0;
}