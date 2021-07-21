#pragma once

#include <queue>

#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"

#include "Eigen/Dense"

#include "group_manager.hpp"


namespace hebi {
namespace experimental {
namespace mobile {

using hebi::trajectory::Trajectory;
using TrajectoryQueue = std::queue<std::shared_ptr<Trajectory>>;

class SE2Point {
  public:
  SE2Point() {}
  SE2Point(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  double theta{std::numeric_limits<double>::quiet_NaN()};

  SE2Point operator+(const SE2Point& rhs) const {
    SE2Point out;
    out.x = this->x + rhs.x;
    out.y = this->y + rhs.y;
    out.theta = this->theta + rhs.theta;
    return out;
  }

  SE2Point operator-(const SE2Point& rhs) const {
    SE2Point out;
    out.x = this->x - rhs.x;
    out.y = this->y - rhs.y;
    out.theta = this->theta - rhs.theta;
    return out;
  }

  void operator+=(const SE2Point& rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->theta += rhs.theta;
  }

};

using Pose = SE2Point;
using Vel = SE2Point;

Pose operator*(const Vel& val, double t) {
  SE2Point out{val.x*t, val.y*t, val.theta*t};
  return out;
}

class Waypoint {
  public:
  double t{std::numeric_limits<double>::quiet_NaN()};
  Pose pos{};
  Vel vel{};
  SE2Point acc{};
};

struct CartesianGoal {
  // Moves with the given relative body velocity for a certain
  // number of seconds, then slows to a stop over a certain number
  // of seconds.  Can be used for continuous replanning, e.g.
  // mobile IO control.
  // \param v target velocity
  // \param rampTime "ramp up" time
  // \param cruiseTime time to drive at velocity v
  // \param slowTime "ramp down" time
  static CartesianGoal createFromVelocity(Vel v, double rampTime, double cruiseTime, double slowTime) {
    Waypoint start;
    start.t = rampTime;
    start.vel = v;

    Waypoint slow;
    slow.t = cruiseTime;
    slow.vel = v;

    Waypoint stop;
    stop.t = cruiseTime + slowTime;
    stop.vel = {0, 0, 0};
    stop.acc = {0, 0, 0};

    return createFromWaypoints({start, slow, stop});
  }

  // Goes to (x, y, theta) in a certain number of seconds.
  static CartesianGoal createFromPosition(Pose p, double goalTime) {
    Waypoint end;

    end.t = goalTime;
    end.pos = p;
    end.vel = {0, 0, 0};
    end.acc = {0, 0, 0};

    return createFromWaypoints({end});
  }

  static CartesianGoal createFromWaypoints(std::vector<Waypoint> waypoints) {
    auto count = waypoints.size();

    VectorXd times(count);
    MatrixXd positions(3, count);
    MatrixXd velocities(3, count);
    MatrixXd accelerations(3, count);

    for (auto idx = 0; idx < count; ++idx) {
      times(idx) = waypoints.at(idx).t;
      positions(0, idx) = waypoints.at(idx).pos.x;
      positions(1, idx) = waypoints.at(idx).pos.y;
      positions(2, idx) = waypoints.at(idx).pos.theta;

      velocities(0, idx) = waypoints.at(idx).vel.x;
      velocities(1, idx) = waypoints.at(idx).vel.y;
      velocities(2, idx) = waypoints.at(idx).vel.theta;

      accelerations(0, idx) = waypoints.at(idx).acc.x;
      accelerations(1, idx) = waypoints.at(idx).acc.y;
      accelerations(2, idx) = waypoints.at(idx).acc.theta;
    }

    return CartesianGoal(times, positions, velocities, accelerations);
  }

  const Eigen::VectorXd& times() const { return times_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& positions() const { return positions_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& velocities() const { return velocities_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& accelerations() const { return accelerations_; }

private:
  CartesianGoal(const Eigen::VectorXd& times,
       const Eigen::Matrix<double, 3, Eigen::Dynamic>& positions,
       const Eigen::Matrix<double, 3, Eigen::Dynamic>& velocities,
       const Eigen::Matrix<double, 3, Eigen::Dynamic>& accelerations)
    : times_(times),
      positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations) {}

  const Eigen::VectorXd times_{0};
  const Eigen::Matrix<double, 3, Eigen::Dynamic> positions_{0, 0};
  const Eigen::Matrix<double, 3, Eigen::Dynamic> velocities_{0, 0};
  const Eigen::Matrix<double, 3, Eigen::Dynamic> accelerations_{0, 0};
};

//////////////////////////////

class MobileBase {

public:

  // Parameters for creating a base
  struct Params: public GroupManager::Params {
  };

  // This is a generic implementation that assumes the use of a wheel-space trajectory
  bool update() {
    // updates dt and last feedback message
    auto ret = group_manager_->update();
    // now update odometry
    auto vel = group_manager_->lastFeedback().getVelocity();
    updateOdometry(vel, group_manager_->dT());

    auto& cmd = group_manager_->pendingCommand();

    while (base_trajectories_.front() && base_trajectories_.front()->getEndTime() < group_manager_->lastTime()) {
      base_trajectories_.pop();
    }

    // go into compliance mode if no trajectory
    if (base_trajectories_.size() == 0) {
      auto size = group_manager_->size();

      double nan = std::numeric_limits<double>::quiet_NaN();
      VectorXd compliantState = VectorXd::Constant(1, size, nan);

      cmd.setPosition(compliantState);
      cmd.setVelocity(compliantState);
      compliantState.setZero();
      cmd.setEffort(compliantState);

      return ret;
    } else {
      auto traj = base_trajectories_.front();

      VectorXd pos, vel, accel;

      // Update command from trajectory
      traj->getState(group_manager_->lastTime(), &pos, &vel, &accel);

      // set velocity to steer along cartesian trajectory

      // Convert from x/y/theta to wheel velocities
      auto wheel_vels = SE2ToWheelVel({pos(0), pos(1), pos(2)}, {vel(0), vel(1), vel(2)});

      // Integrate position using wheel velocities.
      last_wheel_pos_ += wheel_vels * group_manager_->dT();
      cmd.setPosition(last_wheel_pos_);

      // Use velocity from trajectory, converted from x/y/theta into wheel velocities above.
      cmd.setVelocity(wheel_vels);

      return ret;
    }
  }

  bool send() {
    return group_manager_->send();
  }

  virtual SE2Point wheelsToSE2(const Pose& pos, const Eigen::VectorXd& wheel_state) const = 0;
  virtual Eigen::VectorXd SE2ToWheelVel(const Pose& pos, const Vel& vel) const = 0;

  // Get the global odometry state
  Pose getOdometry() const { return global_pose_ + relative_odom_offset_; };

  // Reset the current odometry state
  void setOdometry(Pose p) {
    relative_odom_offset_ = p - getOdometry();
  }

  // Get the maximum velocity in each direction
  // NOTE: consider whether this is guaranteed maximum velocity
  // or maximum velocity if no other directions are used!  Also
  // is this true for any config of legged robots, or is this some
  // "safe" max vel?
  // TODO: potential make this depend on state?
  virtual Vel getMaxVelocity() const = 0;

  // Set the goal.  Goal will be actively commanded until cleared,
  // maintaining the end state of the goal after finishing a smooth
  // trajectory to the goal.
  // Note -- this turns the `Goal` into a `Trajectory` based on the
  // `getMaxVelocity` function implementation
  // TODO: "best effort" -- or return "can't do this"?
  bool setGoal(const CartesianGoal& g) {
    auto baseTrajectories = buildTrajectory(g);
    if (baseTrajectories) {
      base_trajectories_.swap(*baseTrajectories);
      return true;
    }
    return false;
  }

  // When cleared, the base should be passive / compliant if possible.
  void clearGoal() {
    // Taken from https://stackoverflow.com/a/709161
    // clear the trajectory queue
    TrajectoryQueue().swap(base_trajectories_);
  }

  bool goalComplete() {
    if (base_trajectories_.empty())
      return true;
    if (base_trajectories_.back()->getEndTime() < group_manager_->lastTime())
      return true;
    return false;
  };

protected:
  // protected constructor
  MobileBase(std::unique_ptr<GroupManager> gm, Params p) :
    group_manager_(std::move(gm))
  {
    last_wheel_pos_ = group_manager_->lastFeedback().getPosition();
  }

  // A cartesian trajectory is the only thing an individual base
  // implementation must handle.  This should be smoothly moved
  // to from the current state of the system.
  virtual std::unique_ptr<TrajectoryQueue> buildTrajectory(const CartesianGoal& g) = 0;

  virtual void updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) = 0;

  std::unique_ptr<GroupManager> group_manager_{nullptr};

  // These variables should be updated when updateOdometry is called
  Pose global_pose_{0, 0, 0};
  Vel global_vel_{0, 0, 0};
  VectorXd last_wheel_pos_{};

  Vel local_vel_{0, 0, 0};

  TrajectoryQueue base_trajectories_{};

private:
  Pose relative_odom_offset_{0, 0, 0};
};


} // namespace mobile
} // namespace experimental
} // namespace hebi
