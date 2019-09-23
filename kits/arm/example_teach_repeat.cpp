/*
 * This file runs a teach_repeat program with a 6-DoF Arm, allowing you to 
 * physically set waypoints for the arm to then move through when you enter
 * playback mode. This is an example of the arm's ability to get accurate 
 * feedback about its position and replay that for the user. Take note, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>
#include <iostream>

using namespace hebi;
using namespace hebi::experimental; // For all things mobileIO 

struct Waypoint
{
  Eigen::VectorXd positions;
  Eigen::VectorXd vels;
  Eigen::VectorXd accels;
};
  
struct State
{
  int num_modules;
  std::vector<Waypoint> waypoints;
};

void addWaypoint (State& state, const GroupFeedback& feedback, bool stop) {
  printf("Adding a Waypoint.\n");
  
  if (stop) { // stop waypoint
    state.waypoints.push_back(Waypoint {feedback.getPosition(),
                              VectorXd::Constant(state.num_modules, 0),
                              VectorXd::Constant(state.num_modules, 0)});
  }
  else { // through waypoint
    state.waypoints.push_back(Waypoint {feedback.getPosition(),
                      VectorXd::Constant(state.num_modules, std::numeric_limits<double>::quiet_NaN()),
                      VectorXd::Constant(state.num_modules, std::numeric_limits<double>::quiet_NaN())});
  }
}

arm::Goal playWaypoints (State& state) {
  // We know that if we are here, there is at least one waypoint

  // Set up the required variables
  Eigen::MatrixXd target_pos(state.num_modules, state.waypoints.size());
  Eigen::MatrixXd target_vels(state.num_modules, state.waypoints.size());
  Eigen::MatrixXd target_accels(state.num_modules, state.waypoints.size());
  Eigen::VectorXd times(state.waypoints.size());

  // Fill up the relevant matrices
  for (int i = 0; i < state.waypoints.size(); i++)
  {
    // map each waypoint vector to a column in the targets matrix
    target_pos.col(i) << state.waypoints[i].positions;
    target_vels.col(i) << state.waypoints[i].vels;
    target_accels.col(i) << state.waypoints[i].accels;
  }

  // For better motion, we ensure the last waypoint is a stop waypoint
  target_vels.col(state.waypoints.size()-1) << 
                  Eigen::VectorXd::Constant(state.num_modules, 0);
  target_accels.col(state.waypoints.size()-1) << 
                    Eigen::VectorXd::Constant(state.num_modules, 0);

  return arm::Goal(target_pos, target_vels, target_accels);
}



int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Setup Module Family and Module Names
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow",
                                            "Wrist1", "Wrist2", "Wrist3"};

  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  std::unique_ptr<robot_model::RobotModel> model = 
                      robot_model::RobotModel::loadHRDF("hrdf/6-dof_arm.hrdf");

  // Setup Time Variables
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> arm_time = std::chrono::steady_clock::now() - start_time;
  double arm_start_time = arm_time.count();
  
  // Create the Arm Object
  auto arm = arm::Arm::create(arm_start_time, family, names, std::move(model));


  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("HEBI", "Mobile IO");

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Add stop WP\nB2 - Clear waypoints\n"
                  "B3 - Add through WP\nB5 - Playback mode\n"
                  "B6 - Grav comp mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup state variable for mobile device
  auto last_mobile_state = mobile->getState();


  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Teach Repeat Variables
  State state;
  state.num_modules = arm -> robotModel().getDoFCount();

  while(arm->update(arm->currentTime(start_time)))
  {
     // Get latest mobile_state
    auto mobile_state = mobile->getState();
    MobileIODiff diff(last_mobile_state, mobile_state);

    // Buttton B1 - Add Stop Waypoint
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn) {
      addWaypoint(state, arm -> lastFeedback(), true);
    }

    // Button B2 - Clear Waypoints
    if (diff.get(2) == MobileIODiff::ButtonState::ToOn) {
      state.waypoints.clear();
    }

    // Button B3 - Add Through Waypoint
    if (diff.get(3) == MobileIODiff::ButtonState::ToOn) {
      addWaypoint(state, arm -> lastFeedback(), false);
    }

    // Button B5 - Playback Waypoints
    if (diff.get(5) == MobileIODiff::ButtonState::ToOn) {
      if (state.waypoints.size() == 0){
        printf("You have not added any Waypoints!\n");
      } 
      else {
        const arm::Goal playback = playWaypoints(state);
        arm -> setGoal(playback);       
      }
    }

    // Button B6 - Grav Comp Mode
    if (diff.get(6) == MobileIODiff::ButtonState::ToOn) {
      // cancel any goal that is set, returning arm into gravComp mode
      arm -> cancelGoal();
    }

    // Button B8 - End Demo
    if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn) {
      // Clear MobileIO text
      mobile -> clearText();
      return 1;
    }

    // Update to the new last_state
    last_mobile_state = mobile_state;

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}



