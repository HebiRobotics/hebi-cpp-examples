#include <chrono>
#include <mutex>
#include <optional>
#include <stack>
#include <thread>

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
  { { WaypointID::Home,      true},  arm::Goal::createFromPosition(3, eigenize({0, 1.5, 2.8, 3.02, 4.64, 3.14})) },
  { { WaypointID::AbovePick, true},  arm::Goal::createFromPosition(3, eigenize({0, 0.54, 1.85, 3.01, 4.65, 3.14})) },
  { { WaypointID::Pick,      true},  arm::Goal::createFromPosition(3, eigenize({0, 0.22, 1.65, 3.03, 4.69, 3.14})) },
  { { WaypointID::HighDrop,  true},  arm::Goal::createFromPosition(4, eigenize({0, 1.74, 1.58, 1.49, 4.88, 3.14})) },
  { { WaypointID::Home,      false}, arm::Goal::createFromPosition(3, eigenize({0, 1.5, 2.8, 4.59, 4.64, 3.14})) },
  { { WaypointID::AbovePick, false}, arm::Goal::createFromPosition(3, eigenize({0, 0.54, 1.85, 4.58, 4.65, 3.14})) },
  { { WaypointID::Pick,      false}, arm::Goal::createFromPosition(3, eigenize({0, 0.22, 1.65, 4.60, 4.69, 3.14})) },
  { { WaypointID::HighDrop,  false}, arm::Goal::createFromPosition(4, eigenize({0, 1.74, 1.58, 3.06, 4.88, 3.14})) }
};


double currentTime(std::chrono::steady_clock::time_point& start) {
  return (std::chrono::duration<double>(std::chrono::steady_clock::now() - start)).count();
}


///////////////////////////////////

// These values are written from the mobile IO thread and read by the command thread.
// Probably should be encapsulated
// Lock with the command lock!

enum class GripperCommand {
  // Gripper commands
  Reorient, // forward/down
  Toggle, // open/close
};

// The jogging/rotation command state.
struct DemoCommandState {
  float dx{};
  float dy{};
  float dz{};
  float gripper_dtheta{}; // gripper
  std::stack<GripperCommand> gripper_commands;
  std::optional<WaypointID> waypoint;
};

DemoCommandState command_state;

std::mutex command_lock;

///////////////////////////////////

std::unique_ptr<MobileIO> mobile;
// background loop -- gets commands from IO board
// Note -- 'mobile' should be valid before we
// reach this function!
void io_command_loop() {

  auto last_state = mobile->getState();
  while(true) {
    // Get latest mobile_state
    auto state = mobile->getState();
    MobileIODiff diff(last_state, state);

    // Update state stack/data!
    {
      std::lock_guard<decltype(command_lock)> lg(command_lock);
      command_state.dx = state.getAxis(8);
      command_state.dy = state.getAxis(7);
      command_state.dz = state.getAxis(6);
      double gripper_rotation_scale = 0.03;
      command_state.gripper_dtheta = state.getAxis(5) * gripper_rotation_scale;

      if (diff.get(7) == experimental::MobileIODiff::ButtonState::ToOn)
        command_state.gripper_commands.push(GripperCommand::Reorient);
      if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn)
        command_state.gripper_commands.push(GripperCommand::Toggle);
      // B1, B2, B5, B6: Fixed waypoints
      if (diff.get(1) == MobileIODiff::ButtonState::ToOn)
        command_state.waypoint = WaypointID::AbovePick;
      else if (diff.get(2) == MobileIODiff::ButtonState::ToOn)
        command_state.waypoint = WaypointID::Pick;
      else if (diff.get(5) == MobileIODiff::ButtonState::ToOn)
        command_state.waypoint = WaypointID::Home;
      else if (diff.get(6) == MobileIODiff::ButtonState::ToOn)
        command_state.waypoint = WaypointID::HighDrop;
    }
    // Update to the new last_state
    last_state = state;

  }

}

///////////////////////////////////

void clip(double& input, double min_val, double max_val)
{
  input = std::max(std::min(max_val, input), min_val);
}

// Get goal for jogging the robot
arm::Goal get_jogging_goal(const arm::Arm& arm, float dx, float dy, float dz, bool is_gripper_down) {
  // Get current target pos/vel
  // TODO: add vel...
  auto current_joint_position = arm.pendingCommand().getPosition();
 // auto current_joint_vel = arm.pendingCommand().getVelocity();
  auto current_cartesian_position = arm.FK(current_joint_position);
 // const auto& robot_model = arm.robotModel();
 // auto j_end_effector = robot_model.getJEndEffector(current_joint_position);
 // auto current_cartesian_velocity = j_end_effector * current_joint_vel;

  // Desired:
  auto desired_cartesian_position = current_cartesian_position;
  float scale = 0.05f; // How far to go in (some) time...as determined below
  float min_x_low = 0.3;
  float min_x_high = 0.25;
  float max_x_low = 0.575;
  float max_x_high = 0.475;
  float min_y = -0.25;
  float max_y = 0.25;
  float min_z = -0.2;
  float max_z = 0.45;
  // If moving to gripper out, keep further away from body:
  if (!is_gripper_down) {
    min_x_low = 0.32;
    min_x_high = 0.32;
  }
  desired_cartesian_position[0] += dx * scale;
  desired_cartesian_position[1] += -dy * scale;
  desired_cartesian_position[2] += dz * scale;
  clip(desired_cartesian_position[1], min_y, max_y);
  clip(desired_cartesian_position[2], min_z, max_z);
  
  if (desired_cartesian_position[2] < -0.02) {
    clip(desired_cartesian_position[0], min_x_low, max_x_low);
  } else {
    clip(desired_cartesian_position[0], min_x_high, max_x_high);
  }

  // Further limit x if z is low...this prevents arm from hitting robit.

  auto desired_joint_position = arm.solveIK(current_joint_position, desired_cartesian_position, is_gripper_down ? Eigen::Vector3d(0, 0, -1) : Eigen::Vector3d(1, 0, 0));
   
  // Change duration depending on if gripper direction changes... 
  auto max_angle_change = 0.0;
  auto change = (desired_joint_position - current_joint_position);
  for (int i = 0; i < change.size(); ++i)
    max_angle_change = std::max(max_angle_change, std::abs(change[i]));
  double other_scale = 1.2; 
  return arm::Goal::createFromPosition(std::max(max_angle_change * other_scale, 0.5), desired_joint_position);
}


///////////////////////////////////

// main loop -- controls arm

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
  mobile = MobileIO::create("Aster", "mobileIO");

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

    // Gripper gains
    {
      hebi::GroupCommand gripper_gains_cmd(gripper->size());
      while (!gripper_gains_cmd.readGains("gripper-gains.xml")) {
        std::cout << "Failed to read gripper gains!\n";
      }
      while (!gripper->sendCommandWithAcknowledgement(gripper_gains_cmd)) {
        std::cout << "Failed to send gripper gains!\n";
      }
    }
  }

  mobile->sendText("Found gripper!\nLooking for arm...");

  // Create the Arm Object
  auto arm = arm::Arm::create(params);

  while (!arm) {
    std::cout << "Failed to find arm!\n";
    arm = arm::Arm::create(params);
  }

  while (!arm->loadGains("arm-gains.xml")) {
    std::cout << "Failed to send gains to arm!\n";
  }

  // Clear any garbage on screen
  mobile->clearText(); 

  // Display instructions on screen
  mobile->sendText("B1/2/5/6-Waypts\n(Pick, Above, Home, High)\nA6-A8-Jog Tip\nA5-Rotate Gripper\nB7-Gripper Direction\nB8-Open/Close\n");
  mobile->setSnap(5, 0);
  mobile->setSnap(6, 0);

  std::thread io_thread(io_command_loop);

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

  float gripper_theta = 0; 
  while (!arm->update())
  {}   
  gripper_theta = arm->lastFeedback()[5].actuator().position().get();
  
  // Soft start:  
  arm->update();
  std::optional<WaypointID> last_waypoint_id = WaypointID::Home;
  arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));

  // get initial gripper position! 
  while(true) {
    if (!arm->update())
      continue;

    // Gripper direction and state change
    // TODO: change gripper open/close state to use 'aux' instead?

    // Get commands:
    float dx, dy, dz, gripper_dtheta;
    std::optional<WaypointID> new_waypoint;
    bool changed_gripper_direction = false;
    {
      std::lock_guard<decltype(command_lock)> lg(command_lock);
      dx = command_state.dx;
      dy = command_state.dy;
      dz = command_state.dz;
      gripper_dtheta = command_state.gripper_dtheta;
      // Reset!
      command_state.dx = 0;
      command_state.dy = 0;
      command_state.dz = 0;
      command_state.gripper_dtheta = 0;

      while (command_state.gripper_commands.size() > 0) {
        auto tmp_cmd = command_state.gripper_commands.top();
        if (tmp_cmd == GripperCommand::Reorient) {
          is_gripper_down = !is_gripper_down;
          changed_gripper_direction = !changed_gripper_direction;
        } else if (tmp_cmd == GripperCommand::Toggle) {
          is_gripper_open = !is_gripper_open;
        }
        command_state.gripper_commands.pop();
      }

      new_waypoint = command_state.waypoint;
      command_state.waypoint.reset();
    }

    bool has_jog_command = (dx != 0 || dy != 0 || dz != 0);

    if (new_waypoint.has_value()) { // Fixed waypoints
      last_waypoint_id = new_waypoint.value();
      arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));
    } else if (has_jog_command) { // Jogging
      last_waypoint_id.reset();
      arm->setGoal(get_jogging_goal(*arm, dx, dy, dz, is_gripper_down));
    } 
    else if (changed_gripper_direction) { // No new waypoint or jogging, but gripper changed -- need to send a new goal!
      if (last_waypoint_id.has_value()) { // From last waypoint
        arm->setGoal(waypoint_map.at({ last_waypoint_id.value(), is_gripper_down }));
      } else { // From current command -- treat like a jog of the gripper
        arm->setGoal(get_jogging_goal(*arm, 0, 0, 0, is_gripper_down));
      }
    }

    gripper_effort.set(is_gripper_open ? 5 : -5);

    // Set the gripper rotation:
    gripper_theta += gripper_dtheta;
    arm->pendingCommand()[5].actuator().position().set(gripper_theta);
    arm->pendingCommand()[5].actuator().velocity().set(gripper_dtheta);
    arm->pendingCommand()[5].actuator().effort().set(std::numeric_limits<float>::quiet_NaN());

    // Send latest commands to the arm
    arm->send();
    gripper->sendCommand(gripper_command);
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}

