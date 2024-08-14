/**
 * This file is a barebones skeleton of how to setup an arm for use.
 * It demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
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
using namespace experimental;

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/ex_gravity_compensation.cfg.yaml";
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

  // For this demo, we need just the arm
  std::unique_ptr<hebi::experimental::arm::Arm> arm;

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
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update())
  {

    // When no goal is set, the arm automatically returns to grav-comp mode.
    // Thus, when we have an empty control loop, the arm is in grav-comp
    // awaiting further instructions.

    // Send the latest loaded commands to the arm. If no changes are made, 
    // it will send the last loaded command when arm->update was last called
    arm->send();
  }

  return 0;
}



