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
#include <thread>
#include "hebi_util.hpp"

using namespace hebi;
using namespace experimental;

struct Waypoint
{
  Eigen::VectorXd positions;
  Eigen::VectorXd vels;
  Eigen::VectorXd accels;
  double time;
};
  
struct State
{
  int num_modules;
  std::vector<Waypoint> waypoints;
};

void addWaypoint (State& state, double wp_time, const GroupFeedback& feedback, bool stop) {
  printf("Adding a Waypoint.\n");
  
  if (stop) { // stop waypoint
    state.waypoints.push_back(Waypoint {feedback.getPosition(),
                              VectorXd::Constant(state.num_modules, 0),
                              VectorXd::Constant(state.num_modules, 0),
                              wp_time});
  }
  else { // through waypoint
    state.waypoints.push_back(Waypoint {feedback.getPosition(),
                      VectorXd::Constant(state.num_modules, std::numeric_limits<double>::quiet_NaN()),
                      VectorXd::Constant(state.num_modules, std::numeric_limits<double>::quiet_NaN()),
                      wp_time});
  }
}

arm::Goal playWaypoints (State& state) {

  // Set up required variables
  Eigen::MatrixXd target_pos(state.num_modules, state.waypoints.size());
  Eigen::MatrixXd target_vels(state.num_modules, state.waypoints.size());
  Eigen::MatrixXd target_accels(state.num_modules, state.waypoints.size());
  Eigen::VectorXd times(state.waypoints.size());

  // Fill up matrices appropriately
  for (int i = 0; i < state.waypoints.size(); i++)
  {
    target_pos.col(i) << state.waypoints[i].positions;
    target_vels.col(i) << state.waypoints[i].vels;
    target_accels.col(i) << state.waypoints[i].accels;
    times[i] = state.waypoints[i].time;
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

  // For this demo, we need the arm and mobile_io
  std::unique_ptr<hebi::experimental::arm::Arm> arm;
  std::unique_ptr<hebi::util::MobileIO> mobile_io;

  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Create the arm object from the configuration
  arm = hebi::experimental::arm::Arm::create(*example_config);

  // Keep retrying if arm not found
  while (!arm) {
      std::cerr << "Failed to create arm, retrying..." << std::endl;

      // Wait for 1 second before retrying
      std::this_thread::sleep_for(std::chrono::seconds(1));  

      // Retry
      arm = hebi::experimental::arm::Arm::create(*example_config);
  }
  std::cout << "Arm connected." << std::endl;

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the mobile_io object from the configuration
  std::cout << "Waiting for Mobile IO device to come online..." << std::endl;
  mobile_io = createMobileIOFromConfig(*example_config, example_config_file);

  // Keep retrying if Mobile IO not found
  while (mobile_io == nullptr) {
      std::cout << "Couldn't find Mobile IO. Check name, family, or device status..." << std::endl;

      // Wait for 1 second before retrying
      std::this_thread::sleep_for(std::chrono::seconds(1));  

      // Retry
      mobile_io = createMobileIOFromConfig(*example_config, example_config_file);
  }
  std::cout << "Mobile IO connected." << std::endl;

  // Clear any garbage on screen
  mobile_io -> clearText(); 

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Teach Repeat Variables
  State state;
  state.num_modules = arm->robotModel().getDoFCount();

  // Run mode is either "training" or "playback"
  std::string run_mode = "training";

  // Variable to hold slider value for A3
  double slider3 = 0.0;

  // Load travel times from config
  double base_travel_time = example_config->getUserData().floats_.at("base_travel_time");
  double min_travel_time = example_config->getUserData().floats_.at("min_travel_time");

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update())
  {
     // Get latest mobile_state
    bool updated_mobile_io = mobile_io->update(0);

    if (updated_mobile_io)
    {
      if (run_mode == "training")
      {
        // Axis A3 state
        slider3 = mobile_io->getAxis(3);

        // Buttton B1 - Add Stop Waypoint
        if (mobile_io->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
          addWaypoint(state, 
                      base_travel_time + slider3 * (base_travel_time - min_travel_time),
                      arm -> lastFeedback(), 
                      true);
        }

        // Button B2 - Add Through Waypoint
        if (mobile_io->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn) {
          addWaypoint(state, 
                      base_travel_time + slider3 * (base_travel_time - min_travel_time),
                      arm -> lastFeedback(), 
                      false);
        }

        // Button B3 - Toggle to Playback Waypoints
        if (mobile_io->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
          if (state.waypoints.size() <= 1){
            std::cout << "You have not added enough waypoints! You need at least two.\n" << std::endl;
          } 
          else {
            std::cout << "Entering playback mode.\n" << std::endl;
            const arm::Goal playback = playWaypoints(state);
            arm->setGoal(playback);  
            run_mode = "playback"; 
          }
        }

        // Button B4 - Clear Waypoints
        if (mobile_io->getButtonDiff(4) == util::MobileIO::ButtonState::ToOn) {
          std::cout << "Discarding waypoints.\n" << std::endl;
          state.waypoints.clear();
        }
      } 
      else if (run_mode == "playback") 
      {
        // Button B3 - Toggle to Training Mode
        if (mobile_io->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
          std::cout << "Entering training mode.\n" << std::endl;
          arm->cancelGoal();
          run_mode = "training";
        }

        // Replay goal
        if (arm -> atGoal()) {
          const arm::Goal playback = playWaypoints(state);
          arm -> setGoal(playback);
        }
      }

      // Button B8 - End Demo
      if (mobile_io->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile_io->clearText();
        return 1;
      }
    }

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile_io->clearText();

  return 0;
}
