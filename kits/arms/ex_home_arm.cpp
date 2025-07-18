/**
 * This example demonstrates how to set up a HEBI arm using configuration files
 * and command it to move to a predefined home position.
 */

#include "arm/arm.hpp"
#include <chrono>

using namespace hebi;
using namespace experimental;

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/A-2580-06.cfg.yaml";
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

  // Create the arm object from the configuration (retry if not found)
  std::unique_ptr<arm::Arm> arm = arm::Arm::create(*example_config);
  while (!arm) {
    std::cerr << "Failed to create arm, retrying..." << std::endl;
    arm = arm::Arm::create(*example_config);
  }
  std::cout << "Arm connected." << std::endl;

  //////////////////////////
  /// Home Command Setup ///
  //////////////////////////

  Eigen::VectorXd home_position(6);
  home_position << 0.0, 2.09, 2.09, 0.0, 1.57, 0.0;
  double home_duration = 4.0; // seconds
  arm::Goal goal = arm::Goal::createFromPosition(home_duration, home_position);
  arm->update();
  arm->setGoal(goal);
  arm->send();

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update()) {
    // Send the latest loaded commands to the arm. Since the goal is already set,
    // it will send the last loaded command when arm.update() was last called
    arm->send();
  }

  return 0;
}



