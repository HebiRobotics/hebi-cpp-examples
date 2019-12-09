/**
 * This file is a barebones skeleton of how to setup an arm for use.
 * It demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

#include <chrono>
#include <optional>
// #include "lookup.hpp"
// #include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"

using namespace hebi;
using namespace hebi::experimental; // For all things mobileIO 

enum class WaypointID {
  Home,
  AbovePick,
  Pick,
  HighDrop,
};

struct Waypoint {
  Waypoint(WaypointID id, bool is_down) : id_(id), is_down_(is_down) {} 
  WaypointID id_;
  bool is_down_; 
  bool operator< (const Waypoint& rhs) const {
    return id_ < rhs.id_ || (id_ == rhs.id_ && is_down_ < rhs.is_down_);
  }
};

// Joint angle vector

Eigen::VectorXd eigenize(const std::vector<double>& in) {
  Eigen::VectorXd res(in.size());
  for (int i = 0 ; i < in.size(); ++i)
    res[i] = in[i];
  return res;
}

const std::map<Waypoint, arm::Goal> waypoint_map = {
  { { WaypointID::Home,      true},  arm::Goal(3, eigenize({0, 1.5, 2.8, 3.02, 4.64, 3.14})) },
  { { WaypointID::AbovePick, true},  arm::Goal(3, eigenize({0, 0.54, 1.85, 3.01, 4.65, 3.14})) },
  { { WaypointID::Pick,      true},  arm::Goal(3, eigenize({0, 0.22, 1.65, 3.03, 4.69, 3.14})) },
  { { WaypointID::HighDrop,  true},  arm::Goal(4, eigenize({0, 1.74, 1.58, 1.49, 4.88, 3.14})) },
  { { WaypointID::Home,      false}, arm::Goal(3, eigenize({0, 1.5, 2.8, 4.59, 4.64, 3.14})) },
  { { WaypointID::AbovePick, false}, arm::Goal(3, eigenize({0, 0.54, 1.85, 4.58, 4.65, 3.14})) },
  { { WaypointID::Pick,      false}, arm::Goal(3, eigenize({0, 0.22, 1.65, 4.60, 4.69, 3.14})) },
  { { WaypointID::HighDrop,  false}, arm::Goal(4, eigenize({0, 1.74, 1.58, 3.06, 4.88, 3.14})) }
};


double currentTime(std::chrono::steady_clock::time_point& start) {
  return (std::chrono::duration<double>(std::chrono::steady_clock::now() - start)).count();
}

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {"Aster"};
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};

  // Read HRDF file to seutp a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  params.hrdf_file_ = "kits/hrdf/6-dof_arm.hrdf";


  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("Aster", "mobileIO");

  while (!mobile) {
    std::cout << "Failed to find mobile IO!\n";
    mobile = MobileIO::create("Aster", "mobileIO");
  }

  // Clear any garbage on screen
  mobile->clearText(); 

  // Display instructions on screen
  mobile->sendText("Connected to arm control program!\nLooking for gripper...");

  std::shared_ptr<hebi::Group> gripper;
  hebi::GroupCommand gripper_command(1);
  auto& gripper_effort = gripper_command[0].actuator().effort();
  {
    Lookup lookup;
    gripper = lookup.getGroupFromNames({"Aster"}, {"gripperSpool"});

    while (!gripper) {
      std::cout << "Failed to find gripper!\n";
      gripper = lookup.getGroupFromNames({"Aster"}, {"gripperSpool"});
    }
  }

  mobile->sendText("Found gripper!\nLooking for arm...");

  // Setup Time Variables
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> arm_time = std::chrono::steady_clock::now() - start_time;
  double arm_start_time = arm_time.count();
  
  // Create the Arm Object
  auto arm = arm::Arm::create(arm_start_time, params);

  while (!arm) {
    std::cout << "Failed to find arm!\n";
    arm = arm::Arm::create(arm_start_time, params);
  }

  // Clear any garbage on screen
  mobile->clearText(); 

  // Display instructions on screen
  mobile->sendText("B1 - Above Pick\nB2 - Pick\nB5 - Home\nB6 - High\nB7 - Gripper Direction\nB8 - Gripper\n");
//  mobile->setSnap(5, 0);
//  mobile->setSnap(6, 0);

  // Setup instructions
  auto last_state = mobile->getState();

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Single Waypoint Vectors
  auto num_joints = arm -> robotModel().getDoFCount();
  Eigen::VectorXd positions(num_joints);
  double single_time;

  // Time Vector for Waypoints
  int num_wp = 1; 
  Eigen::VectorXd times(num_wp);

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  bool is_gripper_open = true;
  // Which way is the gripper facing?
  bool is_gripper_down = true;

  // Angle of wrist 3
  double gripper_rotation = 3.14159;

  std::optional<WaypointID> last_waypoint_id = WaypointID::Home;
  arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));

  while(arm->update(currentTime(start_time))) {
    // Get latest mobile_state
    auto state = mobile->getState();
    MobileIODiff diff(last_state, state);

    // Button B7/B8: Gripper direction and state change
    bool changed_gripper_direction = false;
    if (diff.get(7) == experimental::MobileIODiff::ButtonState::ToOn) {
      is_gripper_down = !is_gripper_down;
      changed_gripper_direction = true;
    }
    // TODO: change to use 'aux' instead?
    if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn)
      is_gripper_open = !is_gripper_open;

    // B1, B2, B5, B6: Fixed waypoints
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn) {
      last_waypoint_id = WaypointID::AbovePick;
      arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));
    }
    else if (diff.get(2) == MobileIODiff::ButtonState::ToOn) {
      last_waypoint_id = WaypointID::Pick;
      arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));
    }
    else if (diff.get(5) == MobileIODiff::ButtonState::ToOn) {
      last_waypoint_id = WaypointID::Home;
      arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));
    }
    else if (diff.get(6) == MobileIODiff::ButtonState::ToOn) {
      last_waypoint_id = WaypointID::HighDrop;
      arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));
    }
    // No new waypoint, but gripper changed -- need to send a new goal!
    else if (last_waypoint_id.has_value() && changed_gripper_direction)
      arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));

    gripper_effort.set(is_gripper_open ? 5 : -5);

    // Set the gripper rotation:
    double gripper_rotation_scale = 0.1;
    gripper_rotation += state.getAxis(5) * gripper_rotation_scale;
    arm->pendingCommand()[5].actuator().position().set(gripper_rotation);

    // Update to the new last_state
    last_state = state;

    // Send latest commands to the arm
    arm->send();
    gripper->sendCommand(gripper_command);
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}



