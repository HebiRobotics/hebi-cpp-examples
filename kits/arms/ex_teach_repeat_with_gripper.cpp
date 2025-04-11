/*
 * This file runs a teach_repeat program with a 6-DoF Arm with a gripper,
 * allowing you to physically set waypoints for the arm to then move through
 * when you enter playback mode. The particular choice of PID control gains
 * can affect the performance of this demo.
 */

#include <chrono>
#include <iostream>

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"

using namespace hebi;

struct Waypoint {
  Eigen::VectorXd positions;
  Eigen::VectorXd vels;
  Eigen::VectorXd accels;
  arm::Gripper::State gripper_state;
  double time_from_prev;
};

struct State {
  State(int num_modules, arm::Gripper& gripper) : num_modules_(num_modules), gripper_(gripper),
    current_gripper_state_(arm::Gripper::State::Open)
  {
    gripper_.setState(current_gripper_state_);
  }
  int num_modules_{};
  arm::Gripper& gripper_;
  arm::Gripper::State current_gripper_state_{arm::Gripper::State::Open};
  std::vector<Waypoint> waypoints_{};
  void toggleGripper() {
    current_gripper_state_ =
      current_gripper_state_ == arm::Gripper::State::Close ? arm::Gripper::State::Open : arm::Gripper::State::Close;
    gripper_.setState(current_gripper_state_);
  }
  void reset()
  {
    waypoints_.clear();
    current_gripper_state_ = arm::Gripper::State::Open;
    gripper_.setState(current_gripper_state_);
  }
};

// Updates current "goal state" with a new waypoint
void addWaypoint (State& state, double time, const GroupFeedback& feedback, bool stop) {
  std::cout << "Adding a Waypoint.\n";
  if (stop) { // stop waypoint
    state.waypoints_.push_back(Waypoint {feedback.getPosition(),
                              VectorXd::Constant(state.num_modules_, 0),
                              VectorXd::Constant(state.num_modules_, 0),
                              state.current_gripper_state_, time});
  }
  else { // through/flow waypoint
    state.waypoints_.push_back(Waypoint {feedback.getPosition(),
                              VectorXd::Constant(state.num_modules_, std::numeric_limits<double>::quiet_NaN()),
                              VectorXd::Constant(state.num_modules_, std::numeric_limits<double>::quiet_NaN()),
                              state.current_gripper_state_, time});
  }
}

arm::Goal playWaypoints (State& state) {
  int wp_count = state.waypoints_.size();
  int extra_wps = 0;
  // If the last gripper state doesn't match the first, we toggle on the way back to the first:
  if (state.waypoints_.begin()->gripper_state != state.waypoints_.rbegin()->gripper_state)
  {
    extra_wps = 1;
  }

  // Set up required variables
  Eigen::VectorXd times(wp_count + extra_wps);
  Eigen::MatrixXd target_pos(state.num_modules_, wp_count + extra_wps);
  Eigen::MatrixXd target_vels(state.num_modules_, wp_count + extra_wps);
  Eigen::MatrixXd target_accels(state.num_modules_, wp_count + extra_wps);
  Eigen::MatrixXd aux(1, wp_count + extra_wps);

  // Fill up matrices appropriately
  auto t = 0;
  for (int i = 0; i < wp_count; i++)
  {
    const auto& wp = state.waypoints_[i]; 
    t += wp.time_from_prev;
    times[i] = t;
    target_pos.col(i) << wp.positions;
    target_vels.col(i) << wp.vels;
    target_accels.col(i) << wp.accels;
    aux(0, i) = arm::Gripper::StateToDouble(wp.gripper_state);
  }
  if (extra_wps != 0)
  {
    times[wp_count] = t + 0.5;
    const auto& wp = *state.waypoints_.rbegin(); 
    target_pos.col(wp_count) << wp.positions;
    target_vels.col(wp_count) << wp.vels;
    target_accels.col(wp_count) << wp.accels;
    aux(0, wp_count) = arm::Gripper::StateToDouble(state.waypoints_[0].gripper_state);
  }
  return arm::Goal::createFromWaypointsWithAux(times, target_pos, target_vels, target_accels, aux);
}

enum class DemoMode
{
  Training,
  Playback
};

void setTrainingDisplay(util::MobileIO& mobile)
{
  std::string instructions =
      "B1 - Add waypoint (stop)\nB2 - Add waypoint (stop) + toggle gripper\n"
      "B3 - Add waypoint (flow)\nB5 - Toggle training/playback\n"
      "B6 - Clear waypoints\nA3 - Time to waypoint\nB8 - Quit\n";
  std::cout << "\n\n" << instructions;
  mobile.clearText();
  mobile.appendText(instructions);
  mobile.setButtonLabel(1, "STOP");
  mobile.setButtonLabel(2, "GRIP");
  mobile.setButtonLabel(3, "FLOW");
  mobile.setButtonLabel(4, "");
  mobile.setButtonLabel(5, "\u25b6");
  mobile.setButtonLabel(6, "Clear");
  mobile.setButtonLabel(7, "");
  mobile.setButtonLabel(8, "\u274c");
  for (int i = 1; i <= 8; ++i)
    mobile.setAxisLabel(i, "");
  mobile.setAxisLabel(3, "\u231b");
  mobile.setLedColor(0, 0, 255);
}

void setPlaybackDisplay(util::MobileIO& mobile)
{
  std::string instructions =
      "B5 - Toggle training/playback\n"
      "B8 - Quit\n";
  std::cout << "\n\n" << instructions;
  mobile.clearText();
  mobile.appendText(instructions);
  mobile.setButtonLabel(1, "");
  mobile.setButtonLabel(2, "");
  mobile.setButtonLabel(3, "");
  mobile.setButtonLabel(4, "");
  mobile.setButtonLabel(5, "\u23f8");
  mobile.setButtonLabel(6, "");
  mobile.setButtonLabel(7, "");
  mobile.setButtonLabel(8, "\u274c");
  for (int i = 1; i <= 8; ++i)
    mobile.setAxisLabel(i, "");
  mobile.setLedColor(0, 255, 0);
}

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Config Setup ///////
  //////////////////////////

  // Config file path
  const std::string example_config_file = "config/ex_teach_repeat_w_gripper.cfg.yaml";
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
  // Add the gripper
  arm::Gripper* gripper_raw{}; // CAUTION: This memory will be managed by the arm, but we store a typed reference first for use with our state variable below
  {
    auto& user_data = example_config->getUserData();
    auto family = user_data.getString("gripper_family");
    auto name = user_data.getString("gripper_name");
    auto gains_file = user_data.getString("gripper_gains");
    auto close_effort = user_data.getFloat("gripper_close_effort");
    auto open_effort = user_data.getFloat("gripper_open_effort");
    auto gripper = arm::Gripper::create(family, name, close_effort, open_effort);
    while (!gripper) {
      std::cerr << "Failed to create gripper, retrying...\n";
      gripper = arm::Gripper::create(family, name, close_effort, open_effort);
    }
    if (!gripper->loadGains("config/" + gains_file)) {
      std::cerr << "Could not set default gripper gains.\n";
    }
    gripper_raw = gripper.get();
    arm->setEndEffector(std::shared_ptr<arm::EndEffectorBase>(gripper.release()));
  }

  std::cout << "Arm connected." << std::endl;

  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  auto mobile = util::MobileIO::create("Arm", "mobileIO");
  if (!mobile)
  {
    std::cout << "couldn't find mobile IO device!\n";
    return 1;
  }

  // Clear any garbage on screen
  mobile->setLedColor(0, 0, 255);
  mobile->clearText();

  // Demo variables
  bool abort_flag = false;
  auto mode = DemoMode::Training;

  // Print instructions on mobile IO display
  setTrainingDisplay(*mobile); 

  // Populate "last state" for diffs (not really necessary, but helps if
  // program started with button pressed on mobile IO)
  mobile->update();

  /////////////////////////
  /// Main Control Loop ///
  /////////////////////////

  // Teach Repeat Variables
  State state(
    arm->robotModel().getDoFCount(),
    *gripper_raw);

  while (!abort_flag)
  {
    arm->update();

    bool updated_mobile = mobile->update(0);

    if (!updated_mobile)
      std::cout << "Failed to get feedback from mobile I/O; check connection!\n";
    else
    {
      if (mobile->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn)
      {
        abort_flag = true;
        break;
      }

      auto slider3 = mobile->getAxis(3);

      if (mode == DemoMode::Training)
      {
        // Button B1 - Add stop Waypoint
        if (mobile->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn)
        {
          addWaypoint(state, slider3 + 2.5f, arm->lastFeedback(), true);
        }

        // Button B2 - Add stop waypoint (stop) and toggle the gripper
        if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn)
        {
          state.toggleGripper();
          addWaypoint(state, slider3 + 2.5f, arm->lastFeedback(), true);
          // Hang out here for a couple seconds!
          addWaypoint(state, 1.4f, arm->lastFeedback(), true);
        }
    
        // Button B3 - Add Through Waypoint
        if (mobile->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn)
        {
          addWaypoint(state, slider3 + 2.5f, arm->lastFeedback(), false);
        }

        // Button B6 - Clear waypoints
        if (mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn)
        {
          std::cout << "Waypoints cleared\n";
          state.reset();
        }
        
        // Button B5 - Toggle Training/Playback
        if (mobile->getButtonDiff(5) == util::MobileIO::ButtonState::ToOn)
        {
          if (state.waypoints_.size() <= 2){
            std::cout << "At least two waypoint are needed\n";
          } 
          else
          {
            std::cout << "Transitioning to playback mode\n";
            mode = DemoMode::Playback;
            setPlaybackDisplay(*mobile);
            const arm::Goal playback = playWaypoints(state);
            std::cout << "about to set goal\n";
            arm->setGoal(playback);       
            std::cout << "set goal\n";
          }
        }
      }
      else if (mode == DemoMode::Playback)
      {
        // B5 toggle training/playback
        if (mobile->getButtonDiff(5) == util::MobileIO::ButtonState::ToOn)
        {
          std::cout << "Transitioning to training mode\n";
          mode = DemoMode::Training;
          setTrainingDisplay(*mobile);
          // Cancel any goal that is set, returning arm into gravComp mode
          arm->cancelGoal();
        }
        else if (arm->atGoal())
        {
          // replay through the path again once the goal has been reached
          const arm::Goal playback = playWaypoints(state);
          arm->setGoal(playback);       
        }
      }
    }

    // Send latest commands to the arm
    arm->send();
  }

  mobile->resetUI();
  return 0;
}
