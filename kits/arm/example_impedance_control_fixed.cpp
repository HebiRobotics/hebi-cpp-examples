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

The following example is for the "Fixed" demo:
*/

// #include "lookup.hpp"
// #include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>

using namespace hebi;
using namespace experimental; 

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {"HEBIArm-T"};
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};

  // Read HRDF file to seutp a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  params.hrdf_file_ = "kits/arm/hrdf/T-arm.hrdf";

  // Create the Arm Object
  auto arm = arm::Arm::create(params);
  while (!arm) {
    arm = arm::Arm::create(params);
  }

  // Load the gains file that is approriate to the arm
  arm -> loadGains("kits/arm/gains/T-arm.xml");

  // Create and configure the ImpedanceController plugin
  hebi::experimental::arm::PluginConfig impedance_config("ImpedanceController", "ImpedanceController");
  impedance_config.float_lists_["kp"] = {300.0, 300.0, 300.0, 5.0, 5.0, 1.0};
  impedance_config.float_lists_["kd"] = {5.0, 5.0, 5.0, 0.0, 0.0, 0.0};
  impedance_config.float_lists_["ki"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  impedance_config.float_lists_["i_clamp"] = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};
  impedance_config.bools_["gains_in_end_effector_frame"] = true;

  auto impedance_plugin = hebi::experimental::arm::plugin::ImpedanceController::create(impedance_config);
  if (!impedance_plugin) {
    std::cerr << "Failed to create ImpedanceController plugin." << std::endl;
    return -1;
  }

  // Initialize variables used to clear the commanded position and velocity in every cycle
  hebi::GroupCommand& command = arm->pendingCommand();

  auto num_joints = arm->robotModel().getDoFCount();
  Eigen::VectorXd pos_nan(num_joints), vel_nan(num_joints);
  pos_nan.fill(std::numeric_limits<double>::quiet_NaN());
  vel_nan.fill(std::numeric_limits<double>::quiet_NaN());

  // Add the plugin to the arm
  if (!arm->addPlugin(std::move(impedance_plugin))) {
    std::cerr << "Failed to add ImpedanceController plugin to arm." << std::endl;
    return -1;
  }

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Set up MobileIO
  std::unique_ptr<util::MobileIO> mobile = util::MobileIO::create(params.families_[0], "mobileIO");
  if (!mobile)
  {
    std::cout << "couldn't find mobile IO device!\n";
    return 1;
  }
  mobile->setButtonMode(1, util::MobileIO::ButtonMode::Momentary);
  mobile->setButtonLabel(1, "❌");
  mobile->setButtonMode(2, util::MobileIO::ButtonMode::Toggle);
  mobile->setButtonLabel(2, "💪");

  std::string instructions;
  instructions = "                           Fixed demo";
  
  // Clear any garbage on screen
  mobile->clearText(); 

  // Display instructions on screen
  mobile->appendText(instructions); 

  // Setup instructions
  auto last_state = mobile->update();

  std::cout <<  "Commanded gravity-compensated zero force to the arm.\n"
            <<  "  💪 (B2) - Toggles an impedance controller on/off:\n"
            <<  "            ON  - Apply controller based on current position\n"
            <<  "            OFF - Go back to gravity-compensated mode\n"
            <<  "  ❌ (B1) - Exits the demo.\n";

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Flag to indicate when impedance controller is on
  bool controller_on = false;

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update())
  {
    auto updated_mobile = mobile->update(0);

    if (updated_mobile)
    {
      /////////////////
      // Button Presses
      /////////////////

      // Buttton B1 - End demo
      if (mobile->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile->resetUI();
        return 1;
      }

      // Button B2 - Set and unset impedance mode when button is pressed and released, respectively
      if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn) {

        controller_on = true;

        arm->setGoal(arm::Goal::createFromPosition(arm->lastFeedback().getPositionCommand()));
      }
      else if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOff){

        controller_on = false;
      }
    }

    if (!controller_on)
    {
      arm->cancelGoal();
    }

    // Clear all position and velocity commands
    command.setPosition(pos_nan);
    command.setVelocity(vel_nan);

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile->clearText();

  return 0;
}


