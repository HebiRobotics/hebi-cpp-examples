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
namespace arm = hebi::experimental::arm;

enum class GripperState {
  Open, Close
};

struct Waypoint {
  Eigen::VectorXd positions;
  Eigen::VectorXd vels;
  Eigen::VectorXd accels;
  GripperState gripper_state;
  double time_from_prev;
};

static constexpr double gripperEffort(GripperState gs) {
  return gs == GripperState::Open ? 1 : -5;
}

struct State {
  State(int num_modules, hebi::experimental::arm::EndEffectorBase& ee) : num_modules_(num_modules), gripper_(ee),
    current_gripper_state_(GripperState::Open)
  {
    Eigen::VectorXd tmp(1);
    tmp << gripperEffort(current_gripper_state_);
    gripper_.update(tmp);
    // Should check the result here...
    gripper_.send();
  }
  int num_modules_{};
  hebi::experimental::arm::EndEffectorBase& gripper_;
  GripperState current_gripper_state_{GripperState::Open};
  std::vector<Waypoint> waypoints_{};
  void toggleGripper() {
    current_gripper_state_ =
      current_gripper_state_ == GripperState::Close ? GripperState::Open : GripperState::Close;
    Eigen::VectorXd tmp(1);
    tmp << gripperEffort(current_gripper_state_);
    // Should check the result here...
    gripper_.update(tmp);
  }
  void reset()
  {
    waypoints_.clear();
    current_gripper_state_ = GripperState::Open;
    Eigen::VectorXd tmp(1);
    tmp << gripperEffort(current_gripper_state_);
    // Should check the result here...
    gripper_.update(tmp);
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
    aux(0, i) = gripperEffort(wp.gripper_state);
  }
  if (extra_wps != 0)
  {
    times[wp_count] = t + 0.5;
    const auto& wp = *state.waypoints_.rbegin(); 
    target_pos.col(wp_count) << wp.positions;
    target_vels.col(wp_count) << wp.vels;
    target_accels.col(wp_count) << wp.accels;
    aux(0, wp_count) = gripperEffort(state.waypoints_[0].gripper_state);
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
  ///// Arm Setup //////////
  //////////////////////////

  const std::string family = "Arm";

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {family};
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};
  
  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  params.hrdf_file_ = "kits/arm/config/hrdf/A-2085-06G.hrdf";

  // Add the gripper
  std::shared_ptr<hebi::experimental::arm::EndEffectorBase> gripper_shared;
  {
    auto gripper = hebi::experimental::arm::EffortEndEffector<1>::create({family}, {"gripperSpool"});
    if (!gripper)
    {
      std::cout << "couldn't find gripper!\n";
      return 1;
    }
    gripper->loadGains("config/gains/gripper_spool_gains.xml");
    gripper_shared.reset(gripper.release());
    params.end_effector_ = gripper_shared;
  }

  // Create the Arm Object
  auto arm = arm::Arm::create(params);
  while (!arm) {
    std::cout << "waiting for arm...\n";
    arm = arm::Arm::create(params);
  }

  // Load the gains file that is approriate to the arm
  arm->loadGains("kits/arm/config/gains/A-2085-06.xml");


  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  auto mobile = util::MobileIO::create(family, "mobileIO");
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
    *gripper_shared);

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
