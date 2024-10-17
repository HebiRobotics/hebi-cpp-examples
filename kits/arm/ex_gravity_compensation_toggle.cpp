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
#include "hebi_util.hpp"

using namespace hebi;
using namespace experimental;

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/ex_gravity_compensation_toggle.cfg.yaml";
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

  // Retrieve the gravcomp plugin from the arm

  // Get a weak_ptr from the arm API, lock it as a shared_ptr, and then downcast it from a general plugin pointer to a specific plugin pointer
  auto plugin_shared_ptr = arm->getPluginByType<arm::plugin::GravityCompensationEffort>().lock();
  if (!plugin_shared_ptr) {
    std::cerr << "Failed to lock plugin shared_ptr. The plugin may have been destroyed." << std::endl;
    return -1;
  }
  auto gravcomp_plugin_ptr = std::dynamic_pointer_cast<arm::plugin::GravityCompensationEffort>(plugin_shared_ptr);
  if (!gravcomp_plugin_ptr) {
    std::cerr << "Failed to cast plugin to GravityCompensationEffort." << std::endl;
    return -1;
  }

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the mobile_io object from the configuration
  std::cout << "Waiting for Mobile IO device to come online..." << std::endl;
  mobile_io = createMobileIOFromConfig(*example_config);

  // Keep retrying if Mobile IO not found
  while (mobile_io == nullptr) {
      std::cout << "Couldn't find Mobile IO. Check name, family, or device status..." << std::endl;

      // Retry
      mobile_io = createMobileIOFromConfig(*example_config);
  }
  std::cout << "Mobile IO connected." << std::endl;

  // Clear any garbage on screen
  mobile_io->clearText(); 

  // Refresh mobile_io
  auto last_state = mobile_io->update();

  std::cout <<  "Commanded gravity-compensated zero force to the arm.\n"
            <<  "  🌍 (B2) - Toggles the gravity compensation on/off:\n"
            <<  "            ON  - Apply controller \n"
            <<  "            OFF - Disable  controller\n"
            <<  "  ❌ (B1) - Exits the demo.\n";

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

      // Buttton B1 - End demo
      if (mobile_io->getButtonDiff(1) == hebi::util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile_io->resetUI();
        return 1;
      }

      // Button B2 - Set and unset gravcomp mode when button is pressed and released, respectively
      if (mobile_io->getButtonDiff(2) == hebi::util::MobileIO::ButtonState::ToOn) {
        
        // Enable gravcomp  
        gravcomp_plugin_ptr->setEnabled(true);
      }
      else if (mobile_io->getButtonDiff(2) == hebi::util::MobileIO::ButtonState::ToOff){

        // Disable gravcomp
        gravcomp_plugin_ptr->setEnabled(false);
      }
    }
    // Send latest commands to the arm
    arm->send();
  }

  return 0;
}



