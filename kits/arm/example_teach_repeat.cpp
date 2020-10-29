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
using namespace experimental;

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

arm::Goal playWaypoints (State& state, double wp_time) {

  // Set up required variables
  Eigen::VectorXd times(state.waypoints.size());
  Eigen::MatrixXd target_pos(state.num_modules, state.waypoints.size());
  Eigen::MatrixXd target_vels(state.num_modules, state.waypoints.size());
  Eigen::MatrixXd target_accels(state.num_modules, state.waypoints.size());

  // Fill up matrices appropriately
  for (int i = 0; i < state.waypoints.size(); i++)
  {
    times[i] = (i+1) * wp_time;
    target_pos.col(i) << state.waypoints[i].positions;
    target_vels.col(i) << state.waypoints[i].vels;
    target_accels.col(i) << state.waypoints[i].accels;
  }
  return arm::Goal::createFromWaypoints(times, target_pos, target_vels, target_accels);
}


int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {"Arm"};
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};
  
  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  params.hrdf_file_ = "kits/arm/hrdf/A-2085-06.hrdf";  

  // Create the Arm Object
  auto arm = arm::Arm::create(params);
  while (!arm) {
    arm = arm::Arm::create(params);
  }

  // Load the gains file that is approriate to the arm
  arm -> loadGains("kits/arm/gains/A-2085-06.xml");

  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("Arm", "mobileIO");

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

  while(arm->update())
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
        const arm::Goal playback = playWaypoints(state, 2.5);
        arm -> setGoal(playback);       
      }
    }

    // Button B6 - Grav Comp Mode
    if (diff.get(6) == MobileIODiff::ButtonState::ToOn) {
      // Cancel any goal that is set, returning arm into gravComp mode
      arm -> cancelGoal();
    }

    // Button B8 - End Demo
    if (diff.get(8) == MobileIODiff::ButtonState::ToOn) {
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




