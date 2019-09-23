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



int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Setup Module Family and Module Names
  // Use the Scope App to find modules on your network.
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"};
  // Adding a module here but not connecting it causes a segmentation fault?/


  // Setup a RobotModel object for the Arm
  // Here, we are creating a 6-DoF arm
  // This can alternatively be done using a HRDF file 
  // Check docs.hebi.us to learn more about HRDFs and how to load them
  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;
  std::unique_ptr<hebi::robot_model::RobotModel> model(new hebi::robot_model::RobotModel());

  model -> addActuator(ActuatorType::X8_9);
  model -> addBracket(BracketType::X5HeavyRightInside);
  model -> addActuator(ActuatorType::X8_16);
  model -> addLink(LinkType::X5, 0.3, M_PI);
  model -> addActuator(ActuatorType::X8_9);
  model -> addLink(LinkType::X5, 0.3, 0);
  model -> addActuator(ActuatorType::X5_9);
  model -> addBracket(BracketType::X5LightRight);
  model -> addActuator(ActuatorType::X5_4);
  model -> addBracket(BracketType::X5LightLeft);
  model -> addActuator(ActuatorType::X5_4);

  // Setup Time Variables
  // start_time is of the type: std::chrono::time_point<std::chrono::steady_clock> 
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_from_start = std::chrono::steady_clock::now() - start_time;
  double arm_start_time = time_from_start.count();
  
  // Create the Arm Object itself
  auto arm = hebi::arm::Arm::create(arm_start_time, family, names, std::move(model));

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update(arm->currentTime(start_time)))
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



