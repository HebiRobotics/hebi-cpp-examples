#include "group_trajectory_follower.hpp"
#include "lookup.hpp"

namespace hebi {
namespace experimental {

std::unique_ptr<GroupTrajectoryFollower> GroupTrajectoryFollower::create(const GroupTrajectoryFollower::Params& params) {

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
  return std::unique_ptr<GroupTrajectoryFollower>(new GroupTrajectoryFollower(params.get_current_time_s_, group));
}
  
bool GroupTrajectoryFollower::loadGains(const std::string& gains_file)
{
  hebi::GroupCommand gains_cmd(group_->size());
  if (!gains_cmd.readGains(gains_file))
    return false;

  return group_->sendCommandWithAcknowledgement(gains_cmd);
}

bool GroupTrajectoryFollower::update() {
  double t = get_current_time_s_();

  // Time must be monotonically increasing!
  if (t < last_time_)
    return false;

  dt_ = t - last_time_;
  last_time_ = t;

  if (!group_->getNextFeedback(feedback_))
    return false;

  // Define aux state so end effector can be updated
  Eigen::VectorXd aux(0);

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

bool GroupTrajectoryFollower::send() {
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

void GroupTrajectoryFollower::setGoal(const Goal& goal) {
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

  // Create new trajectory
  trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                waypoint_times, positions, &velocities, &accelerations);
  trajectory_start_time_ = last_time_;
}

double GroupTrajectoryFollower::goalProgress() const {
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    return t_traj / trajectory_->getDuration();
  }
  // No current goal!
  return 0.0;
}

void GroupTrajectoryFollower::cancelGoal() {
  trajectory_ = nullptr;
  trajectory_start_time_ = std::numeric_limits<double>::quiet_NaN();
}

} // namespace experimental
} // namespace hebi