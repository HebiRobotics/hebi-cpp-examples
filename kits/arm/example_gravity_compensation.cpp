/**
 * This file is a barebones skeleton of how to setup an arm for use.
 * It demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

// #include "lookup.hpp"
// #include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
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
  params.families_ = {"Aster"}; //[change back]
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};

  // Read HRDF file to seutp a RobotModel object for the 6-DoF Arm
  params.hrdf_file_ = "kits/hrdf/6-dof_arm.hrdf";

  // Setup Time Variables
  // auto start_time = std::chrono::steady_clock::now();
  // std::chrono::duration<double> arm_time = std::chrono::steady_clock::now() - start_time;
  
  // Create the Arm Object
  auto arm = arm::Arm::create(params);

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



