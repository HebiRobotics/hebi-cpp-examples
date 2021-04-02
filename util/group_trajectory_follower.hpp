#pragma once

// HEBI C++ API components
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"

#include "arm/goal.hpp"

namespace hebi {
namespace experimental {

using arm::Goal;

class GroupTrajectoryFollower {

public:

  //////////////////////////////////////////////////////////////////////////////
  // Setup functions
  //////////////////////////////////////////////////////////////////////////////

  // Parameters for creating an arm
  struct Params {
    // The family and names passed to the "lookup" function to find modules
    // Both are required.
    std::vector<std::string> families_;
    std::vector<std::string> names_;
    // How long a command takes effect for on the robot before expiring.
    int command_lifetime_ = 100;
    // Loop rate, in Hz.  This is how fast the arm update loop will nominally
    // run.
    double control_frequency_ = 200.f;

    // A function pointer that returns a double representing the current time in
    // seconds. (Can be overloaded to use, e.g., simulator time)
    //
    // The default value uses the steady clock wall time.
    std::function<double()> get_current_time_s_ = []() {
      using clock = std::chrono::steady_clock;
      static const clock::time_point start_time = clock::now();
      return (std::chrono::duration<double>(clock::now() - start_time)).count();
    }; 
  };

  // Creates an "Trajectory Follower" object, and puts it into a "weightless" no-goal control
  // mode.
  static std::unique_ptr<GroupTrajectoryFollower> create(const Params& params);

  // Loads gains from the given .xml file, and sends them to the module. Returns
  // false if the gains file could not be found, if these is a mismatch in
  // number of modules, or the modules do not acknowledge receipt of the gains.
  bool loadGains(const std::string& gains_file);

  //////////////////////////////////////////////////////////////////////////////
  // Accessors
  //////////////////////////////////////////////////////////////////////////////

  // Returns the number of modules / DoF in the arm
  size_t size() const { return group_->size(); }

  // Returns the internal group. Not necessary for most use cases.
  const Group& group() const { return *group_; }

  // Returns the currently active internal trajectory. Not necessary for most
  // use cases.
  // Returns 'nullptr' if there is no active trajectory.
  const trajectory::Trajectory* trajectory() const { return trajectory_.get(); }

  // Returns the command last computed by update, or an empty command object
  // if "update" has never successfully run. The returned command can be
  // modified as desired before it is sent to the robot with the send function.
  GroupCommand& pendingCommand() { return command_; }
  const GroupCommand& pendingCommand() const { return command_; }

  // Returns the last feedback obtained by update, or an empty feedback object
  // if "update" has never successfully run.
  const GroupFeedback& lastFeedback() const { return feedback_; }

  //////////////////////////////////////////////////////////////////////////////
  // Main loop functions
  //
  // Typical usage:
  //
  // while(true) {
  //   arm->update();
  //   arm->send();
  // }
  //////////////////////////////////////////////////////////////////////////////

  // Updates feedback and generates the basic command for this timestep.
  // To retrieve the feedback, call `getLastFeedback()` after this call.
  // You can modify the command object after calling this.
  //
  // Returns 'false' on a connection problem; true on success.
  bool update();

  // Sends the command last computed by "update" to the robot arm.  Any user
  // modifications to the command are included.
  bool send();

  //////////////////////////////////////////////////////////////////////////////
  // Goals
  // 
  // A goal is a desired (joint angle) position that the arm should reach, and
  // optionally information about the time it should reach that goal at and the
  // path (position, velocity, and acceleration waypoints) it should take to
  // get there.
  //
  // The default behavior when a goal is set is for the arm to plan and begin
  // executing a smooth motion from its current state to this goal, with an
  // internal heuristic that defines the time at which it will reach the goal.
  // This immediately overrides any previous goal that was set.
  //
  // If there is no "active" goal the arm is set into a mode where it is
  // actively controlled to be approximately weightless, and can be moved around
  // by hand easily.  This is the default state when the arm is created.
  //
  // After reaching the goal, the arm continues to be commanded with the final
  // joint state of the set goal, and is _not_ implicitly returned to a
  // "weightless" mode.
  //
  // A goal may also define "aux" states to be sent to an end effector
  // associated with the arm.  In this case, the end effector states are
  // treated as "step functions", immediately being commanded at the timestamp
  // of the waypoint they are associated with.  An empty "aux" goal or "NaN"
  // defines a "no transition" at the given waypoint.
  //////////////////////////////////////////////////////////////////////////////

  // Set the current goal waypoint(s), immediately replanning to these
  // location(s) and optionally end effector states.
  // Goal is a commanded position / velocity.
  void setGoal(const Goal& goal);

  // Returns the progress (from 0 to 1) of the current goal, per the last
  // update call.
  //
  // If we have reached the goal, progress is "1".  If there is no active goal,
  // or we have just begun, progress is "0".
  double goalProgress() const;

  // Have we reached the goal?  If there is no goal, returns 'false'
  bool atGoal() const { return goalProgress() >= 1.0; }

  // Cancels any active goal, returning to a "weightless" state which does not
  // actively command position or velocity.
  void cancelGoal();

protected:
  std::function<double()> get_current_time_s_;
  double last_time_;
  double dt_{ std::numeric_limits<double>::quiet_NaN() };
  std::shared_ptr<Group> group_;

  hebi::GroupFeedback feedback_;
  hebi::GroupCommand command_;

  // Private arm constructor
  GroupTrajectoryFollower(
      std::function<double()> get_current_time_s,
      std::shared_ptr<Group> group):
    get_current_time_s_(get_current_time_s),
    last_time_(get_current_time_s()),
    group_(group),
    pos_(Eigen::VectorXd::Zero(group->size())),
    vel_(Eigen::VectorXd::Zero(group->size())),
    accel_(Eigen::VectorXd::Zero(group->size())),
    feedback_(group->size()),
    command_(group->size()) {}

private:
  // The joint angle trajectory for reaching the current goal.
  std::shared_ptr<trajectory::Trajectory> trajectory_;
  double trajectory_start_time_{ std::numeric_limits<double>::quiet_NaN() };
  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
};

}
}