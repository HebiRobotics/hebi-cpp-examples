/**
 * Mobile IO Control
 * An example for setting up your arm for simple control from a mobile io devoce
 * to pre-programmed waypoints.
 */

/*
CAUTION: 
This example uses waypoints containing fixed joint angles, which is a bad idea if your actuators have large wind-up. 
The correct way to store waypoints is by using se3 coordinates, and converting them to joint positions using our IK functions.
*/

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>
#include "hebi_util.hpp"

using namespace hebi;
using namespace experimental; 

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/ex_mobile_io_control.cfg.yaml";
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
  std::unique_ptr<arm::Arm> arm;
  std::unique_ptr<hebi::util::MobileIO> mobile_io;

  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Create the arm object from the configuration
  arm = arm::Arm::create(*example_config);

  // Keep retrying if arm not found
  while (!arm) {
      std::cerr << "Failed to create arm, retrying..." << std::endl;

      // Retry
      arm = arm::Arm::create(*example_config);
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

      // Retry
      mobile_io = createMobileIOFromConfig(*example_config, example_config_file);
  }
  std::cout << "Mobile IO connected." << std::endl;

  // Clear any garbage on screen
  mobile_io->clearText(); 

  // Refresh mobile_io
  auto last_state = mobile_io->update();

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Waypoints
  auto num_joints = arm->robotModel().getDoFCount();
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().float_lists_.at("waypoint_1").data(), example_config->getUserData().float_lists_.at("waypoint_1").size()));
  waypoints.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().float_lists_.at("waypoint_2").data(), example_config->getUserData().float_lists_.at("waypoint_2").size()));
  waypoints.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().float_lists_.at("waypoint_3").data(), example_config->getUserData().float_lists_.at("waypoint_3").size()));

  // Travel time
  double travel_time = example_config->getUserData().floats_.at("travel_time");

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update())
  {
    auto updated_mobile_io = mobile_io->update(0);

    if (updated_mobile_io)
    {
      /////////////////
      // Button Presses
      /////////////////

      // BN - Waypoint N (N = 1, 2 , 3)
      for (int button = 1; button <= 3; button++)
      {
        if (mobile_io->getButtonDiff(button) == hebi::util::MobileIO::ButtonState::ToOn) {
          arm -> setGoal(arm::Goal::createFromPosition(travel_time, waypoints.at(button-1)));
        }
      }

      // Button B6 - Grav Comp Mode
      if (mobile_io->getButtonDiff(6) == hebi::util::MobileIO::ButtonState::ToOn) {
        // cancel any goal that is set, returning arm into gravComp mode
        arm -> cancelGoal();
      }

      // Button B8 - End Demo
      if (mobile_io->getButtonDiff(8) == hebi::util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile_io->resetUI();
        return 1;
      }
    }

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile_io->clearText();

  return 0;
}



