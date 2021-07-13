#pragma once

// HEBI C++ API components
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"
#include "lookup.hpp"

#include "trajectory_manager.hpp"
#include "arm/goal.hpp"

namespace hebi {
namespace experimental {

using arm::Goal;

class GroupTrajectoryManager {

public:

  //////////////////////////////////////////////////////////////////////////////
  // Setup functions
  //////////////////////////////////////////////////////////////////////////////

  // Creates an "Trajectory Follower" object, and puts it into a "weightless" no-goal control
  // mode.
  static std::unique_ptr<GroupTrajectoryManager> create(const GroupManager::Params& params);

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
  const double dT() const { return dt_; }
  const double lastTime() const { return last_time_; }

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
  GroupTrajectoryManager(
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


std::unique_ptr<GroupTrajectoryManager> GroupTrajectoryManager::create(const GroupManager::Params& params) {

  // Get the group (scope the lookup object so it is destroyed
  // immediately after the lookup operation)
  std::shared_ptr<Group> group;
  {
    Lookup lookup;
    group = lookup.getGroupFromNames(params.families_, params.names_);
  }
  if (!group) {
    std::cout << "Could not create arm! Check that family and names match actuators on the network.\n";
    return nullptr;
  }

  // Set parameters
  if (!group->setCommandLifetimeMs(params.command_lifetime_)) {
    std::cout << "Could not set command lifetime on group; check that it is valid.\n";
    return nullptr;
  }
  if (!group->setFeedbackFrequencyHz(params.control_frequency_)) {
    std::cout << "Could not set feedback frequency on group; check that it is valid.\n";
    return nullptr;
  }

  // Try to get feedback -- if we don't get a packet in the first N times,
  // something is wrong
  int num_attempts = 0;

  // We need feedback, so we can plan trajectories if that gets called before the first "update"
  GroupFeedback feedback(group->size());
  while (!group->getNextFeedback(feedback)) {
    if (num_attempts++ > 10) {
      std::cout << "Could not communicate with robot; check network connection.\n";
      return nullptr;
    }
  } 

  // Note: once ROS moves up to C++14, we can change this to "make_unique".
  return std::unique_ptr<GroupTrajectoryManager>(new GroupTrajectoryManager(params.get_current_time_s_, group));
}
  
bool GroupTrajectoryManager::loadGains(const std::string& gains_file)
{
  hebi::GroupCommand gains_cmd(group_->size());
  if (!gains_cmd.readGains(gains_file))
    return false;

  return group_->sendCommandWithAcknowledgement(gains_cmd);
}

bool GroupTrajectoryManager::update() {
  double t = get_current_time_s_();

  // Time must be monotonically increasing!
  if (t < last_time_)
    return false;

  dt_ = t - last_time_;
  last_time_ = t;

  if (!group_->getNextFeedback(feedback_))
    return false;

  // Update command from trajectory
  if (trajectory_) {
    // If we have an active trajectory to our goal, use this.
    // Note -- this applies even if we are past the end of it;
    // we just stay with last state.
    // (trajectory_start_time_ should not be nan here!)
    double t_traj = t - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    trajectory_->getState(t_traj, &pos_, &vel_, &accel_);
  } else {
    pos_.setConstant(std::numeric_limits<double>::quiet_NaN());
    vel_.setConstant(std::numeric_limits<double>::quiet_NaN());
    accel_.setConstant(0.0);
  }
  command_.setPosition(pos_);
  command_.setVelocity(vel_);

  return true;
}

bool GroupTrajectoryManager::send() {
  return group_->sendCommand(command_);
}

// TODO: think about adding customizability, or at least more intelligence for
// the default heuristic.
Eigen::VectorXd getWaypointTimes(
  const Eigen::MatrixXd& positions,
  const Eigen::MatrixXd& velocities,
  const Eigen::MatrixXd& accelerations) {

  double rampTime = 1.2;

  size_t num_waypoints = positions.cols();

  Eigen::VectorXd times(num_waypoints);
  for (size_t i = 0; i < num_waypoints; ++i)
    times[i] = rampTime * (double)i;

  return times;
}

void GroupTrajectoryManager::setGoal(const Goal& goal) {
  int num_joints = goal.positions().rows();

  // If there is a current trajectory, use the commands as a starting point;
  // if not, replan from current feedback.
  Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);

  // Replan if these is a current trajectory:
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    trajectory_->getState(t_traj, &curr_pos, &curr_vel, &curr_accel);
  } else {
    curr_pos = feedback_.getPosition();
    curr_vel = feedback_.getVelocity();
    // (accelerations remain zero)
  }

  int num_waypoints = goal.positions().cols() + 1;

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // Initial state
  positions.col(0) = curr_pos;
  velocities.col(0) = curr_vel;
  accelerations.col(0) = curr_accel;

  // Copy new waypoints
  positions.rightCols(num_waypoints - 1) = goal.positions();
  velocities.rightCols(num_waypoints - 1) = goal.velocities();
  accelerations.rightCols(num_waypoints - 1) = goal.accelerations();

  // Get waypoint times
  Eigen::VectorXd waypoint_times(num_waypoints);
  // If time vector is empty, automatically determine times
  if (goal.times().size() == 0) {
    waypoint_times = getWaypointTimes(positions, velocities, accelerations);
  } else {
    waypoint_times(0) = 0;
    waypoint_times.tail(num_waypoints - 1) = goal.times();
  }

  /*
  std::cout << "Times:\n" << waypoint_times << std::endl;
  std::cout << "Pos:\n" << positions << std::endl;
  std::cout << "Vel:\n" << velocities << std::endl;
  std::cout << "Acc:\n" << accelerations << std::endl;
  */

  // Create new trajectory
  trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                waypoint_times, positions, &velocities, &accelerations);
  trajectory_start_time_ = last_time_;
}

double GroupTrajectoryManager::goalProgress() const {
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    return t_traj / trajectory_->getDuration();
  }
  // No current goal!
  return 0.0;
}

void GroupTrajectoryManager::cancelGoal() {
  trajectory_ = nullptr;
  trajectory_start_time_ = std::numeric_limits<double>::quiet_NaN();
}

} // namespace experimental
} // namespace hebi
