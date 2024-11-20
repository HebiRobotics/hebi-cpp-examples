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
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/ex_teach_repeat.cfg.yaml";
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

  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Create the arm object from the configuration, and retry if not found
  auto arm = arm::Arm::create(*example_config);
  while (!arm) {
    std::cerr << "Failed to create arm, retrying..." << std::endl;
    arm = arm::Arm::create(*example_config);
  }
  std::cout << "Arm connected." << std::endl;


  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  std::unique_ptr<util::MobileIO> mobile = util::MobileIO::create("Arm", "mobileIO");
  if (!mobile)
  {
    std::cout << "couldn't find mobile IO device!\n";
    return 1;
  }

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Add stop WP\nB2 - Clear waypoints\n"
                  "B3 - Add through WP\nB5 - Playback mode\n"
                  "B6 - Grav comp mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile->appendText(instructions); 

  // Setup state variable for mobile device
  auto last_mobile_state = mobile->update();


  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Teach Repeat Variables
  State state;
  state.num_modules = arm->robotModel().getDoFCount();

  while(arm->update())
  {
     // Get latest mobile_state
    bool updated_mobile = mobile->update(0);

    if (!updated_mobile)
      std::cout << "Failed to get feedback from mobile I/O; check connection!\n";
    else
    {
      // Buttton B1 - Add Stop Waypoint
      if (mobile->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
        addWaypoint(state, arm -> lastFeedback(), true);
      }

      // Button B2 - Clear Waypoints
      if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn) {
        state.waypoints.clear();
      }

      // Button B3 - Add Through Waypoint
      if (mobile->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
        addWaypoint(state, arm -> lastFeedback(), false);
      }

      // Button B5 - Playback Waypoints
      if (mobile->getButtonDiff(5) == util::MobileIO::ButtonState::ToOn) {
        if (state.waypoints.size() <= 1){
          printf("You have not added enough waypoints!\n");
        } 
        else {
          const arm::Goal playback = playWaypoints(state, 2.5);
          arm->setGoal(playback);       
        }
      }

      // Button B6 - Grav Comp Mode
      if (mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn) {
        // Cancel any goal that is set, returning arm into gravComp mode
        arm->cancelGoal();
      }

      // Button B8 - End Demo
      if (mobile->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile->clearText();
        return 1;
      }
    }

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile->clearText();

  return 0;
}
