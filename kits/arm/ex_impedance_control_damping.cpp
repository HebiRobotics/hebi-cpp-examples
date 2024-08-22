/**
In these examples we will implement various hybrid motion-force controllers using the impedance control plugin, which can be used for a wide variety of 
applications.
Impedance control is BEST SUITED for assigning free, rigid and springy behaviour, along/about each different axis.
While this is perfectly useful for:
- Having a selectively compliant end-effector,
- Switching between fixed and free behaviour to simulate (mostly) rigid constraints, and
- Allowing human intervention for automated operations by separating controls across different axes,
any applications involving more salient control of the forces (as more complex functions with flexible inputs) should use our force control plugin. See ex_force_control_demoname.cpp.

This comprises the following demos:
- Fixed: A task-space pose controller implemented entirely using force control via the (PID) impedance controller.
- Cartesian: Locks onto a particular end-effector position while having some compliant orientation.
- Gimbal: A gimbal that locks a specific end-effector orientation, while keeping the rest of the arm compliant.
- Floor: The end-effector is free to move but can't travel below a virtual floor. To further simulate sliding on the floor, see force_control example.
- Damping: The end-effector behaves as 3-different damped systems (overdamped, critically damped, and underdamped), at 3 different heights.

The following example is for the "Damping" demo:
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
  const std::string example_config_file = "config/ex_impedance_control_damping.cfg.yaml";
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

  // Ideally, in the impedance control demos, positions and velocities must not be commanded

  // Initialize variables used to clear the commanded position and velocity in every cycle
  // Get the pending command pointer
  hebi::GroupCommand& command = arm->pendingCommand();

  // Create nan vectors for positions and velocities
  auto num_joints = arm->robotModel().getDoFCount();
  Eigen::VectorXd pos_nan(num_joints), vel_nan(num_joints);
  pos_nan.fill(std::numeric_limits<double>::quiet_NaN());
  vel_nan.fill(std::numeric_limits<double>::quiet_NaN());

  // Retrieve the impedance controller plugin from the arm

  // Get a weak_ptr from the arm API, lock it as a shared_ptr, and then downcast it from a general plugin pointer to a specific plugin pointer
  auto plugin_shared_ptr = arm->getPluginByType<arm::plugin::ImpedanceController>().lock();
  if (!plugin_shared_ptr) {
    std::cerr << "Failed to lock plugin shared_ptr. The plugin may have been destroyed." << std::endl;
    return -1;
  }
  auto impedance_plugin_ptr = std::dynamic_pointer_cast<arm::plugin::ImpedanceController>(plugin_shared_ptr);
  if (!impedance_plugin_ptr) {
    std::cerr << "Failed to cast plugin to ImpedanceController." << std::endl;
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
            <<  "  💪 (B2) - Toggles an impedance controller on/off:\n"
            <<  "            ON  - Apply controller based on current position\n"
            <<  "            OFF - Go back to gravity-compensated mode\n"
            <<  "  ❌ (B1) - Exits the demo.\n";

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Meters above the base for overdamped, critically damped, and underdamped cases respectively
  std::vector<double> lower_limits = example_config->getUserData().getFloatList("lower_limits"); 

  // State variable for current mode: 0 for overdamped, 1 for crtically damped, 2 for underdamped, -1 for free
  int mode = -1;
  int prevmode = -1;

  // Kp and Kd in different modes
  std::vector<Eigen::VectorXd> damping_kp, damping_kd;

  // 0: Overdamped
  damping_kp.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().getFloatList("overdamped_kp").data(), example_config->getUserData().getFloatList("overdamped_kp").size()));
  damping_kd.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().getFloatList("overdamped_kd").data(), example_config->getUserData().getFloatList("overdamped_kd").size()));

  // 1: Critically damped
  damping_kp.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().getFloatList("critically_damped_kp").data(), example_config->getUserData().getFloatList("critically_damped_kp").size()));
  damping_kd.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().getFloatList("critically_damped_kd").data(), example_config->getUserData().getFloatList("critically_damped_kd").size()));

  // 2: Underdamped
  damping_kp.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().getFloatList("underdamped_kp").data(), example_config->getUserData().getFloatList("underdamped_kp").size()));
  damping_kd.push_back(Eigen::Map<const Eigen::VectorXd>(example_config->getUserData().getFloatList("underdamped_kd").data(), example_config->getUserData().getFloatList("underdamped_kd").size()));

  // Flag to indicate when impedance controller is on
  bool controller_on = false;

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

      // Button B2 - Set and unset impedance mode when button is pressed and released, respectively
      if (mobile_io->getButtonDiff(2) == hebi::util::MobileIO::ButtonState::ToOn) {

        controller_on = true;

        arm->setGoal(arm::Goal::createFromPosition(arm->lastFeedback().getPosition()));
      }
      else if (mobile_io->getButtonDiff(2) == hebi::util::MobileIO::ButtonState::ToOff){

        controller_on = false;
      }
    }

    if (!controller_on)
    {
      arm->cancelGoal();
      mode = -1; // Free
      prevmode = -1; 
    }
    else
    {
      // Use forward kinematics to calculate pose of end-effector
      Eigen::Vector3d ee_position_curr;
      Eigen::Matrix3d ee_orientation_curr;
      arm->FK(arm->lastFeedback().getPosition(), ee_position_curr, ee_orientation_curr);

      // Assign mode based on current position
      for (int i = 0; i < static_cast<int>(lower_limits.size()); i++)
      {
        if (ee_position_curr(2) > lower_limits[i])
        {
          mode = i;
        }
      }

      // Change gains only upon mode switches
      if (mode != prevmode && mode >= 0)
      {
        impedance_plugin_ptr->setKp(damping_kp.at(mode));
        impedance_plugin_ptr->setKd(damping_kd.at(mode));
        std::cout << "Mode: " << mode << std::endl;
      }
      prevmode = mode;
    }

    // Clear all position and velocity commands
    command.setPosition(pos_nan);
    command.setVelocity(vel_nan);

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile_io->clearText();

  return 0;
}



