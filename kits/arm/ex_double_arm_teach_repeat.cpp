/*
 * This file runs a teach_repeat program with two 6-DoF Arms with grippers,
 * allowing you to physically set waypoints for the arms to then move through
 * when you enter playback mode. The particular choice of PID control gains
 * can affect the performance of this demo.
 */

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "double_arm_teach_repeat_state.hpp"
#include "double_arm_teach_repeat_display.hpp"

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"

namespace arm = hebi::experimental::arm;

namespace hebi {
namespace examples {

enum class DemoMode
{
  TrainingLeft,
  TrainingRight,
  TrainingNone,
  Load,
  Save,
  PlaybackSingle,
  PlaybackRepeat
};

// We don't follow the same pattern for the save/load
// values as the button ordering, so we can use the
// right column for the "back" and "exit" state.
// This mapping handles the converstion from 
// mobile IO button -> displayed value.
const std::map<int, std::string> SaveLoadButtonMap
{
  {1, "1"},
  {2, "3"},
  {3, "2"},
  {4, "4"},
  {5, "5"},
  {7, "6"},
};

template <class CanLoadGains>
bool tryLoadGains(CanLoadGains& obj, const std::string& gains) {
  for (int i = 0; i < 3; ++i)
  {
    if (obj->loadGains(gains))
      return true;
  }
  return false;
}

} // namespace hebi
} // namespace examples

using namespace hebi;
using namespace hebi::examples;

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  const std::string family = "Arm";
  const std::string l_family = "LeftArm";
  const std::string r_family = "RightArm";

  // Setup Module Family and Module Names
  arm::Arm::Params l_params, r_params;
  l_params.families_ = {l_family};
  r_params.families_ = {r_family};
  l_params.names_ = r_params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};
  
  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  l_params.hrdf_file_ = "kits/arm/hrdf/A-2085-06G.hrdf";
  r_params.hrdf_file_ = "kits/arm/hrdf/A-2085-06G.hrdf";

  // Add the gripper
  std::shared_ptr<hebi::experimental::arm::EndEffectorBase> l_gripper_shared, r_gripper_shared;
  {
    auto l_gripper = hebi::experimental::arm::EffortEndEffector<1>::create({l_family}, {"gripperSpool"});
    auto r_gripper = hebi::experimental::arm::EffortEndEffector<1>::create({r_family}, {"gripperSpool"});
    while (!l_gripper)
    {
      std::cout << "Waiting for left gripper!\n";
      l_gripper = hebi::experimental::arm::EffortEndEffector<1>::create({l_family}, {"gripperSpool"});
    }
    while (!r_gripper)
    {
      std::cout << "Waiting for right gripper!\n";
      r_gripper = hebi::experimental::arm::EffortEndEffector<1>::create({r_family}, {"gripperSpool"});
    }
    if (!tryLoadGains(l_gripper, "kits/arm/gains/gripper_spool_gains.xml"))
    {
      std::cerr << "Could not load gains for left gripper!\n";
      return 1;
    }
    if (!tryLoadGains(r_gripper, "kits/arm/gains/gripper_spool_gains.xml"))
    {
      std::cerr << "Could not load gains for right gripper!\n";
      return 1;
    }
    l_gripper_shared.reset(l_gripper.release());
    r_gripper_shared.reset(r_gripper.release());
    l_params.end_effector_ = l_gripper_shared;
    r_params.end_effector_ = r_gripper_shared;
  }

  // Create the Arm Object
  auto l_arm = arm::Arm::create(l_params);
  auto r_arm = arm::Arm::create(r_params);
  while (!l_arm) {
    std::cout << "waiting for left arm...\n";
    l_arm = arm::Arm::create(l_params);
  }
  while (!r_arm) {
    std::cout << "waiting for right arm...\n";
    r_arm = arm::Arm::create(r_params);
  }

  // Load the gains file that is approriate to the arm
  if (!tryLoadGains(l_arm, "kits/arm/gains/A-2085-06.xml"))
  {
    std::cerr << "Could not load gains for left arm!\n";
    return 1;
  }
  if (!tryLoadGains(r_arm, "kits/arm/gains/A-2085-06.xml"))
  {
    std::cerr << "Could not load gains for left arm!\n";
    return 1;
  }

  if (!l_arm->update() || !r_arm->update())
  {
    std::cerr << "could not get initial feedback from arms!\n";
    return 1;
  }

  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  auto mobile = util::MobileIO::create(family, "mobileIO");
  while (!mobile)
  {
    std::cout << "Waiting for mobile IO device!\n";
    mobile = util::MobileIO::create(family, "mobileIO");
  }
  // Initialize cleanly!
  mobile->resetUI();

  // Demo variables
  bool abort_flag = false;
  auto mode = DemoMode::TrainingNone;

  // Print instructions on mobile IO display
  MobileIoState current_state = trainingDisplay(true);
  bool mobile_io_view_updated = current_state.sendTo(*mobile, 3);

  // Populate "last state" for diffs (not really necessary, but helps if
  // program started with button pressed on mobile IO)
  mobile->update();

  /////////////////////////
  /// Main Control Loop ///
  /////////////////////////

  // Teach Repeat Variables
  DemoState state(*l_arm, *l_gripper_shared, *r_arm, *r_gripper_shared);

  // initialize          
  state.stopArm(*l_arm, state.left_.current_gripper_state_);
  state.stopArm(*r_arm, state.right_.current_gripper_state_);

  int mobile_io_stale_count = 0;

  while (!abort_flag)
  {
    l_arm->update();
    r_arm->update();

    bool updated_mobile = mobile->update(0);

    if (!updated_mobile)
    {
      ++mobile_io_stale_count;
      if (mobile_io_stale_count >= 5)
        std::cerr << "Failed to get feedback from mobile I/O in last 5 attempts; check connection!\n";
    }
    else
    {
      if (mobile->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn)
      {
        abort_flag = true;
        std::cout << "Quitting!\n";
        break;
      }

      // Coming back from away -- resend state in case we've reset the app for some reason:
      if (mobile_io_stale_count >= 5 || !mobile_io_view_updated)
      {
        // Just send once, though; don't want to hang things too long...
        mobile_io_view_updated = current_state.sendTo(*mobile);
      }
      mobile_io_stale_count = 0;

      auto lr_select = mobile->getAxis(3);
      auto speed = mobile->getAxis(4);
      bool repeat = mobile->getAxis(5) <= 0;

      if (mode == DemoMode::TrainingLeft ||
          mode == DemoMode::TrainingRight ||
          mode == DemoMode::TrainingNone)
      {
        // INTERNAL TRANSITIONS:

        // Update L/R mode based on slider
        if (lr_select > 0.3) {
          mobile_io_view_updated = mobile->setAxisLabel(3, "Left") && mobile_io_view_updated;
          current_state.setAxisLabel(3, "Left");
          if (mode == DemoMode::TrainingRight) {
            state.stopArm(*r_arm, state.right_.current_gripper_state_);
          }
          if (mode != DemoMode::TrainingLeft) {
            l_arm->cancelGoal();
          }
          mode = DemoMode::TrainingLeft;
        } else if (lr_select < -0.3) {
          mobile_io_view_updated = mobile->setAxisLabel(3, "Right") && mobile_io_view_updated;
          current_state.setAxisLabel(3, "Right");
          if (mode == DemoMode::TrainingLeft) {
            state.stopArm(*l_arm, state.left_.current_gripper_state_);
          }
          if (mode != DemoMode::TrainingRight) {
            r_arm->cancelGoal();
          }
          mode = DemoMode::TrainingRight;
        } else {
          mobile_io_view_updated = mobile->setAxisLabel(3, "None") && mobile_io_view_updated;
          current_state.setAxisLabel(3, "None");
          if (mode == DemoMode::TrainingLeft) {
            state.stopArm(*l_arm, state.left_.current_gripper_state_);
          }
          if (mode == DemoMode::TrainingRight) {
            state.stopArm(*r_arm, state.right_.current_gripper_state_);
          }
          mode = DemoMode::TrainingNone;
        }

        // Update Single/Repeat mode based on slider
        if (repeat) {
          mobile_io_view_updated = mobile->setAxisLabel(5, "Repeat") && mobile_io_view_updated;
          mobile->setAxisLabel(5, "Repeat");
        } else {
          mobile_io_view_updated = mobile->setAxisLabel(5, "Single") && mobile_io_view_updated;
          mobile->setAxisLabel(5, "Single");
        }

        // Button B1 - Add stop Waypoint
        if (mobile->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn)
        {
          state.addWaypoint(speed, true);
        }

        // Button B2 - Toggle the gripper; state will apply for next waypoint.
        if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn)
        {
          if (mode == DemoMode::TrainingLeft)
            state.toggleGripper(LeftRight::Left);
          else if (mode == DemoMode::TrainingRight)
            state.toggleGripper(LeftRight::Right);
        }
    
        // Button B3 - Add Through Waypoint
        if (mobile->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn)
        {
          state.addWaypoint(speed, false);
        }

        // Button B4 - Clear waypoints
        if (mobile->getButtonDiff(4) == util::MobileIO::ButtonState::ToOn)
        {
          std::cout << "Waypoints cleared\n";
          state.reset();
        }

        // Button B5 - Save Waypoints
        if (mobile->getButtonDiff(5) == util::MobileIO::ButtonState::ToOn) {
          std::cout << "Transitioning to save waypoints menu\n";
          mode = DemoMode::Save;
          // Stop the arm if not already stopped:
          if (mode == DemoMode::TrainingLeft)
            state.stopArm(*l_arm, state.left_.current_gripper_state_);
          else if (mode == DemoMode::TrainingLeft)
            state.stopArm(*r_arm, state.right_.current_gripper_state_);
          current_state = saveDisplay(SaveLoadButtonMap, state.listSavedWaypoints(SaveLoadButtonMap));
          mobile_io_view_updated = current_state.sendTo(*mobile);
        }
        
        // Button B6 - Load Waypoints
        else if (mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn) {
          std::cout << "Transitioning to load waypoints menu\n";
          mode = DemoMode::Load;
          current_state = loadDisplay(SaveLoadButtonMap, state.listSavedWaypoints(SaveLoadButtonMap));
          mobile_io_view_updated = current_state.sendTo(*mobile);
        }

        // Button B7 - Toggle Training/Playback
        else if (mobile->getButtonDiff(7) == util::MobileIO::ButtonState::ToOn) {
          if (state.numWaypoints() <= 2) {
            std::cout << "At least two waypoint are needed\n";
          } 
          else {
            std::cout << "Transitioning to playback mode\n";
            if (repeat)
              mode = DemoMode::PlaybackRepeat;
            else
              mode = DemoMode::PlaybackSingle;
            current_state = playbackDisplay();
            mobile_io_view_updated = current_state.sendTo(*mobile);
            state.playWaypoints(repeat);
          }
        }
      }
      else if (mode == DemoMode::PlaybackSingle || mode == DemoMode::PlaybackRepeat)
      {
        // B7 toggle training/playback
        if (mobile->getButtonDiff(7) == util::MobileIO::ButtonState::ToOn)
        {
          std::cout << "Transitioning to training mode\n";
          current_state = trainingDisplay(mode == DemoMode::PlaybackRepeat);
          mobile_io_view_updated = current_state.sendTo(*mobile);
          mode = DemoMode::TrainingNone;
          // Cancel any old goal that is set, stopping the arm so it holds position
          state.stopArm(*l_arm, state.left_.current_gripper_state_);
          state.stopArm(*r_arm, state.right_.current_gripper_state_);
        }
        else
        {
          // Replay through the path again once the goal has been reached
          // (these should reach the goal at the same time, since this is based
          // on commands)
          if (mode == DemoMode::PlaybackRepeat && (l_arm->atGoal() || r_arm->atGoal()))
          {
            state.playWaypoints(true);
          }
        }
      }
      else if (mode == DemoMode::Load)
      {
        bool loaded_waypoints = false;
        for (auto kvp : SaveLoadButtonMap)
        {
          if (mobile->getButtonDiff(kvp.first) == util::MobileIO::ButtonState::ToOn)
          {
            loaded_waypoints = state.loadWaypoints(kvp.second) == DemoState::LoadResult::Success;
            break;
          }
        }

        // B6 Go back to training mode
        if (loaded_waypoints || mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn)
        {
          std::cout << "Transitioning to training mode\n";
          current_state = trainingDisplay(repeat);
          mobile_io_view_updated = current_state.sendTo(*mobile);
          mode = DemoMode::TrainingNone;
          state.stopArm(*l_arm, state.left_.current_gripper_state_);
          state.stopArm(*r_arm, state.right_.current_gripper_state_);
        }
      }
      else if (mode == DemoMode::Save)
      {
        bool saved_waypoints = false;
        for (auto kvp : SaveLoadButtonMap)
        {
          if (mobile->getButtonDiff(kvp.first) == util::MobileIO::ButtonState::ToOn)
          {
            saved_waypoints = state.saveWaypoints(kvp.second);
            break;
          }
        }

        // B6 Go back to training mode
        if (saved_waypoints || mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn)
        {
          std::cout << "Transitioning to training mode\n";
          current_state = trainingDisplay(repeat);
          mobile_io_view_updated = current_state.sendTo(*mobile);
          mode = DemoMode::TrainingNone;
        }
      }
    }

    // Send latest commands to the arm
    l_arm->send();
    r_arm->send();
  }

  mobile->resetUI();
  return 0;
}
