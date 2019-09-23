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
#include "util/mobile_io.hpp"
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
  //// MobileIO Setup //////
  //////////////////////////

  // std::vector<std::string> mobile_family = {"HEBI"};
  // std::vector<std::string> mobile_name = {"Mobile IO"};

  // Lookup mobile_lookup;
  // auto group = mobile_lookup.getGroupFromNames(mobile_family, mobile_names);

  std::unique_ptr<hebi::experimental::MobileIO> mobile = hebi::experimental::MobileIO::create("HEBI", "Mobile IO");

  // mobile.setButtonOutput

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Single waypoint vectors
  // size_t num_joints = model -> getDoFCount();
  // TODO: Why isn't the above line working?
  int num_joints = 6;
  Eigen::VectorXd positions(num_joints);
  Eigen::VectorXd vels(num_joints);
  Eigen::VectorXd accels(num_joints);
  double single_time;

  // Multiple waypoint matrices
  // Eigen::MatrixXd positionsM(num_joints,num_wp);
  // Eigen::MatrixXd velsM(num_joints,num_wp);
  // Eigen::MatrixXd accelsM(num_joints,num_wp);

  // Time Vector for Waypoints
  int num_wp = 1; 
  Eigen::VectorXd times(num_wp);


  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  auto last_state = mobile->getState();
  while(arm->update(arm->currentTime(start_time)))
  {
    auto state = mobile->getState();
    hebi::experimental::MobileIODiff diff(last_state, state);
    if (diff.get(1) == hebi::experimental::MobileIODiff::ButtonState::ToOn)
    //if (mobile -> getState().getButton(1)) // If Button B1 is pressed
    {
      printf("B1!\n");
      positions << 0, 0, 0, 0, 0, 0;
      // vels << 0, 0, 0, 0, 0, 0;
      // accels << 0, 0, 0, 0, 0, 0;
      single_time = 3;
      arm -> setGoal(hebi::arm::Goal(single_time, positions, vels, accels));
    }


    // if (mobile -> getState().getButton(2)){
    if (diff.get(2) == hebi::experimental::MobileIODiff::ButtonState::ToOn)
    {
      printf("B2!\n");
      positions << M_PI/4, M_PI/3, 2*M_PI/3, M_PI/3 ,M_PI/2,0;
      // vels << 0, 0, 0, 0, 0, 0;
      // accels << 0, 0, 0, 0, 0, 0;
      single_time = 3;
      arm -> setGoal(hebi::arm::Goal(single_time, positions, vels, accels));
    }

    if (diff.get(5) == hebi::experimental::MobileIODiff::ButtonState::ToOn)
    {
      printf("B5!\n");
      positions << -M_PI/4, M_PI/3, 2*M_PI/3, M_PI/3 ,M_PI/2,0;
      // vels << 0, 0, 0, 0, 0, 0;
      // accels << 0, 0, 0, 0, 0, 0;
      single_time = 3;
      arm -> setGoal(hebi::arm::Goal(single_time, positions, vels, accels));
    }

    // if (mobile -> getState().getButton(5))
    if (diff.get(6) == hebi::experimental::MobileIODiff::ButtonState::ToOn)
    {
      printf("B6! Returning to Grav Comp\n");
      arm -> cancelGoal();
    }


    // When no goal is set, the arm automatically returns to grav-comp mode.
    // Thus, when we have an empty control loop, the arm is in grav-comp
    // awaiting further instructions.

    // Send the latest loaded commands to the arm. If no changes are made, 
    // it will send the last loaded command when arm->update was last called
    arm->send();
    last_state = state;
  }

  return 0;
}



