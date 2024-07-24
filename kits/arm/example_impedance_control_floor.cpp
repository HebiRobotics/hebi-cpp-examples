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

The following example is for the "Floor" demo:
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

  // NOTE: Angle wraparound is an unresolved issue which can lead to unstable behaviour for any case involving rotational positional control. 
  //       Make sure that the rotational gains are high enough to prevent large angular errors (greater than pi/2). The gains provided in these examples are well behaved.
  //       Interacting with the end-effector in these examples is perfectly safe.
  //       However, ensure that nothing prevents the wrist's actuators from moving, and DO NOT place your fingers between them. 
  
  hebi::experimental::arm::PluginConfig impedance_config("ImpedanceController", "ImpedanceController");
  // impedance_config.float_lists_["kp"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // impedance_config.float_lists_["kd"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // impedance_config.float_lists_["ki"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // impedance_config.float_lists_["i_clamp"] = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};
  impedance_config.bools_["gains_in_end_effector_frame"] = true;

  impedance_config.float_lists_["kp"] = {300.0, 300.0, 300.0, 5.0, 5.0, 1.0};
  impedance_config.float_lists_["kd"] = {5.0, 5.0, 5.0, 0.0, 0.0, 0.0};
  impedance_config.float_lists_["ki"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  impedance_config.float_lists_["i_clamp"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

  // Keep a reference to the ImpedanceController plugin
  // You will have to do this if you intend to reconfigure the impedance control after adding it.
  auto impedance_plugin_ptr = impedance_plugin.get();

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
  instructions = "                           Floor demo";
  
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

  // Initialize floor demo variables
  double floor_level = 0.0;
  double floor_buffer = 0.01; // 1cm

  // Initialize floor demo flags
  bool floor_command_flag = false; // Indicates whether or not to command floor stiffness goals
  bool cancel_command_flag = false; // Indicates whether or not to cancel goals

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

        arm->setGoal(arm::Goal::createFromPosition(arm->lastFeedback().getPosition()));

        // Store current height as floor level, for floor demo

        // Use forward kinematics to find end-effector pose
        Eigen::Vector3d ee_position0;
        Eigen::Matrix3d ee_orientation0;
        arm->FK(arm->lastFeedback().getPosition(), ee_position0, ee_orientation0);

        // Give a little margin to floor level
        floor_level = ee_position0(2) - floor_buffer;

        // Update flags to indicate having left the floor
        cancel_command_flag = true;
      }
      else if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOff){

        controller_on = false;
      }
    }

    if (!controller_on)
    {
      arm->cancelGoal();
    }
    else
    {
      // Use forward kinematics to calculate pose of end-effector
      Eigen::Vector3d ee_position_curr, ee_position_floor;
      Eigen::VectorXd joint_position_floor(num_joints);
      Eigen::Matrix3d ee_orientation_curr;
      arm->FK(arm->lastFeedback().getPosition(), ee_position_curr, ee_orientation_curr);

      // Snap goal to floor if end-effector is at or below floor, only when it first reaches the floor
      if(ee_position_curr(2) <= floor_level && floor_command_flag)
      {
          // Set surface stiffness of floor
          // impedance_config.float_lists_["kp"] = {300.0, 300.0, 300.0, 5.0, 5.0, 1.0};
          // impedance_config.float_lists_["kd"] = {5.0, 5.0, 5.0, 0.0, 0.0, 0.0};
          // impedance_config.float_lists_["ki"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          // impedance_config.float_lists_["i_clamp"] = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};

          // std::vector<float> new_kp = {300.0, 300.0, 300.0, 5.0, 5.0, 5.0};
          // std::vector<float> new_kd = {5.0, 5.0, 5.0, 5.0, 5.0, 1.0};
          // std::vector<float> new_ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          // std::vector<float> new_i_clamp = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};

          std::cout << "1" << std::endl;
          std::cout << impedance_plugin_ptr->kp()(2) << std::endl;
          std::cout << impedance_plugin_ptr->kd()(2) << std::endl;
          std::cout << impedance_plugin_ptr->ki()(2) << std::endl;
          std::cout << impedance_plugin_ptr->iClamp().size() << std::endl;
          std::cout << impedance_plugin_ptr->gainsInEndEffectorFrame() << std::endl;

          Eigen::VectorXd new_kp(6), new_kd(6), new_ki(6), new_i_clamp(6);
          new_kp << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_kd << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_ki << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_i_clamp << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          bool new_gains = false;

          std::vector<bool> vals;
          vals.push_back(impedance_plugin_ptr->setKp(new_kp));
          vals.push_back(impedance_plugin_ptr->setKd(new_kd));
          vals.push_back(impedance_plugin_ptr->setKi(new_ki));
          vals.push_back(impedance_plugin_ptr->setIClamp(new_i_clamp));
          impedance_plugin_ptr->setGainsInEndEffectorFrame(new_gains);
          std::cout << vals.at(0) << vals.at(1) << vals.at(2) << vals.at(3) << std::endl;

          std::cout << "2" << std::endl;
          std::cout << impedance_plugin_ptr->kp()(2) << std::endl;
          std::cout << impedance_plugin_ptr->kd()(2) << std::endl;
          std::cout << impedance_plugin_ptr->ki()(2) << std::endl;
          std::cout << impedance_plugin_ptr->iClamp().size() << std::endl;
          std::cout << impedance_plugin_ptr->gainsInEndEffectorFrame() << std::endl;

          Eigen::VectorXd new_kp2, new_kd2, new_ki2, new_i_clamp2;
          bool new_gains2 = true;

          std::vector<bool> vals2;
          vals2.push_back(impedance_plugin_ptr->setKp(new_kp2));
          vals2.push_back(impedance_plugin_ptr->setKd(new_kd2));
          vals2.push_back(impedance_plugin_ptr->setKi(new_ki2));
          vals2.push_back(impedance_plugin_ptr->setIClamp(new_i_clamp2));
          impedance_plugin_ptr->setGainsInEndEffectorFrame(new_gains2);
          std::cout << vals2.at(0) << vals2.at(1) << vals2.at(2) << vals2.at(3) << std::endl;

          std::cout << "3" << std::endl;
          std::cout << impedance_plugin_ptr->kp()(2) << std::endl;
          std::cout << impedance_plugin_ptr->kd()(2) << std::endl;
          std::cout << impedance_plugin_ptr->ki()(2) << std::endl;
          std::cout << impedance_plugin_ptr->iClamp().size() << std::endl;
          std::cout << impedance_plugin_ptr->gainsInEndEffectorFrame() << std::endl;

          Eigen::VectorXd new_kp3(6), new_kd3(7), new_ki3(5), new_i_clamp3(6);
          new_kp3 << 2.0, 2.0, -2.0, 2.0, 2.0, 2.0;
          new_kd3 << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_ki3 << 2.0, 2.0, 2.0, 2.0, 2.0;
          new_i_clamp3 << 2.0, -2.0, 2.0, 2.0, 2.0, 2.0;
          bool new_gains3 = false;

          std::vector<bool> vals3;
          vals3.push_back(impedance_plugin_ptr->setKp(new_kp3));
          vals3.push_back(impedance_plugin_ptr->setKd(new_kd3));
          vals3.push_back(impedance_plugin_ptr->setKi(new_ki3));
          vals3.push_back(impedance_plugin_ptr->setIClamp(new_i_clamp3));
          impedance_plugin_ptr->setGainsInEndEffectorFrame(new_gains3);
          std::cout << vals3.at(0) << vals3.at(1) << vals3.at(2) << vals3.at(3) << std::endl;

          std::cout << "4" << std::endl;
          std::cout << impedance_plugin_ptr->kp()(2) << std::endl;
          std::cout << impedance_plugin_ptr->kd()(2) << std::endl;
          std::cout << impedance_plugin_ptr->ki()(2) << std::endl;
          std::cout << impedance_plugin_ptr->iClamp().size() << std::endl;
          std::cout << impedance_plugin_ptr->gainsInEndEffectorFrame() << std::endl;

          Eigen::VectorXd new_kp4(6), new_kd4(6), new_ki4(6), new_i_clamp4(6);
          new_kp4 << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_kd4 << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_ki4 << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          new_i_clamp4 << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          bool new_gains4 = false;

          std::vector<bool> vals4;
          vals4.push_back(impedance_plugin_ptr->setKp(new_kp4));
          vals4.push_back(impedance_plugin_ptr->setKd(new_kd4));
          vals4.push_back(impedance_plugin_ptr->setKi(new_ki4));
          vals4.push_back(impedance_plugin_ptr->setIClamp(new_i_clamp4));
          impedance_plugin_ptr->setGainsInEndEffectorFrame(new_gains4);
          std::cout << vals4.at(0) << vals4.at(1) << vals4.at(2) << vals4.at(3) << std::endl;

          std::cout << "5" << std::endl;
          std::cout << impedance_plugin_ptr->kp()(2) << std::endl;
          std::cout << impedance_plugin_ptr->kd()(2) << std::endl;
          std::cout << impedance_plugin_ptr->ki()(2) << std::endl;
          std::cout << impedance_plugin_ptr->iClamp().size() << std::endl;
          std::cout << impedance_plugin_ptr->gainsInEndEffectorFrame() << std::endl;

          // std::cout << impedance_plugin->kp()(2);
          // impedance_plugin_ptr->applyParameter("kp", new_kp);
          // impedance_plugin_ptr->applyParameter("kd", new_kd);
          // impedance_plugin_ptr->applyParameter("ki", new_ki);
          // impedance_plugin_ptr->applyParameter("i_clamp", new_i_clamp);

          // impedance_plugin_ptr->applyParameters(impedance_config, {"gains_in_end_effector_frame", "kp", "kd", "ki", "i_clamp"});
          // impedance_plugin_ptr->applyParameterImpl("kp", new_kp);

          // Snap current pose to floor
          ee_position_floor = ee_position_curr;
          ee_position_floor(2) = floor_level;

          // Use inverse kinematics to calculate appropriate joint positions
          joint_position_floor = arm->solveIK(arm->lastFeedback().getPosition(), ee_position_floor, ee_orientation_curr);

          // Set snapped pose as goal
          arm->setGoal(arm::Goal::createFromPosition(0.001, joint_position_floor)); // Time is very small to make less complex trajectories

          std::cout << "goal set\n";

          // Update flags to indicate being in contact with the floor
          floor_command_flag = false;
          cancel_command_flag = true;
      }
      // Cancel goal if end-effector is above the floor, only when it leaves the floor
      else if (ee_position_curr(2) > floor_level and cancel_command_flag)
      {
          // Set stiffness in air
          impedance_config.float_lists_["kp"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          impedance_config.float_lists_["kd"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          impedance_config.float_lists_["ki"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          impedance_config.float_lists_["i_clamp"] = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};

          // Cancel goal to move freely
          arm->cancelGoal();

          // Update flags to indicate having left the floor
          cancel_command_flag = false;
          floor_command_flag = true;
      }
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



