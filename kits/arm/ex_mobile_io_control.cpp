/**
 * Mobile IO Control
 * An example for setting up your arm for simple control from a mobile io device
 * to pre-programmed waypoints.
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


  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Create the arm object from the configuration, and retry if not found
  std::unique_ptr<arm::Arm> arm = arm::Arm::create(*example_config);
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

  std::string instructions;
  instructions = ("B1 - Waypoint 1\nB2 - Waypoint 2\n"
                  "B3 - Waypoint 3\n"
                  "B6 - Grav comp mode\nB8 - Quit\n");
  // Clear any garbage on screen
  mobile_io->clearText();

  // Display instructions on screen
  mobile_io->appendText(instructions);

  // Setup instructions
  auto last_state = mobile_io->update();

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Waypoints
  auto num_joints = arm->robotModel().getDoFCount();
  const auto user_data = example_config->getUserData();

  std::vector<Eigen::VectorXd> waypoints {
    waypoints.push_back(util::stdToEigenXd(user_data.getFloatList("waypoint_1"))),
    waypoints.push_back(util::stdToEigenXd(user_data.getFloatList("waypoint_2"))),
    waypoints.push_back(util::stdToEigenXd(user_data.getFloatList("waypoint_3")));
  };

  // Travel time
  double travel_time = user_data.getFloat("travel_time");

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update())
  {
    auto updated_mobile_io = mobile_io->update(0);

    if (!updated_mobile_io)
      std::cout << "Failed to get feedback from mobile I/O; check connection!\n";
    else
    {
      /////////////////
      // Button Presses
      /////////////////

      // BN - Waypoint N (N = 1, 2, 3)
      for (int button = 1; button <= 3; button++)
      {
        if (mobile_io->getButtonDiff(button) == util::MobileIO::ButtonState::ToOn) {
          arm -> setGoal(arm::Goal::createFromPosition(travel_time, waypoints.at(button-1)));
        }
      }

      // Button B6 - Grav Comp Mode
      if (mobile_io->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn) {
        // cancel any goal that is set, returning arm into gravComp mode
        arm -> cancelGoal();
      }

      // Button B8 - End Demo
      if (mobile_io->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
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

