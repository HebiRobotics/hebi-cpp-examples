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
#include <thread>
#include "hebi_util.hpp"

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

  // For this demo, we need the arm and mobile_io
  std::unique_ptr<hebi::experimental::arm::Arm> arm;
  std::unique_ptr<hebi::util::MobileIO> mobile_io;

  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Create the arm object from the configuration
  arm = hebi::experimental::arm::Arm::create(*example_config);

  // Keep retrying if arm not found
  while (!arm) {
      std::cerr << "Failed to create arm, retrying..." << std::endl;

      // Wait for 1 second before retrying
      std::this_thread::sleep_for(std::chrono::seconds(1));  

      // Retry
      arm = hebi::experimental::arm::Arm::create(*example_config);
  }
  std::cout << "Arm connected." << std::endl;

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the mobile_io object from the configuration
  std::cout << "Waiting for Mobile IO device to come online..." << std::endl;
  mobile_io = createMobileIOFromConfig(*example_config, example_config_file);

  // Keep retrying if Mobile IO not found
  while (mobile_io == nullptr) {
      std::cout << "Couldn't find Mobile IO. Check name, family, or device status..." << std::endl;

      // Wait for 1 second before retrying
      std::this_thread::sleep_for(std::chrono::seconds(1));  

      // Retry
      mobile_io = createMobileIOFromConfig(*example_config, example_config_file);
  }
  std::cout << "Mobile IO connected." << std::endl;

  // Clear any garbage on screen
  mobile_io->clearText();

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Different modes it can be in (when in none, then automatic grav_comp)
  bool softstart = true;
  bool ar_mode = false;
  
  // Load user data from config
  Eigen::VectorXd home_position(arm -> robotModel().getDoFCount());
  home_position = Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().float_lists_.at("home_position").data(), example_config->getUserData().float_lists_.at("home_position").size());  
  double soft_start_time = example_config->getUserData().floats_.at("soft_start_time");
  double xyz_scale = example_config->getUserData().floats_.at("xyz_scale");

  // Command the softstart to home position
  arm -> update();
  arm -> setGoal(arm::Goal::createFromPosition(soft_start_time, home_position)); // take 4 seconds
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
      
      if (updated_mobile_io)
      {
        // Button B1 - Return to home position
        if (mobile_io->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
            ar_mode = false;
            arm -> setGoal(arm::Goal::createFromPosition(4, home_position));
        }

        // Button B3 - Start AR Control
        if (mobile_io->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
          xyz_phone_init << mobile_io -> getLastFeedback().mobile().arPosition().get().getX(),
                            mobile_io -> getLastFeedback().mobile().arPosition().get().getY(),
                            mobile_io -> getLastFeedback().mobile().arPosition().get().getZ();
          std::cout << xyz_phone_init << std::endl;
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
      Eigen::Vector3d xyz_scale_vec;
      xyz_scale_vec << 1, 1, 2;
      Eigen::Vector3d xyz_target = xyz_home + (xyz_scale * xyz_scale_vec.array() *
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
  mobile_io->resetUI();

  return 0;
}
