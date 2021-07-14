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

using std::queue;
using std::shared_ptr;
using hebi::trajectory::Trajectory;

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

class CartesianTrajectory;

struct CartesianGoal {
  // Moves with the given relative body velocity for a certain
  // number of seconds, then slows to a stop over a certain number
  // of seconds.  Can be used for continuous replanning, e.g.
  // mobile IO control.
  // \param v target velocity
  // \param cruiseTime time to drive at velocity v
  // \param slowTime "ramp down" time
  static CartesianGoal createFromVelocity(Vel v, double cruiseTime, double slowTime) {
    Waypoint start;
    start.t = 0.1;
    start.vel = v;

    Waypoint slow;
    slow.t = cruiseTime;
    slow.vel = v;

    Waypoint stop;
    stop.t = cruiseTime + slowTime;
    stop.vel.x = 0;
    stop.vel.y = 0;
    stop.vel.theta = 0;

    return createFromWaypoints({start, slow, stop}, true);
  }

  // Goes to (x, y, theta) in a certain number of seconds.
  // \param isBaseFrame is "true" if this is relative to current body frame,
  // and "false" for using odometry coordinates.
  static CartesianGoal createFromPosition(Pose p, double goalTime, bool isBaseFrame) {
    Waypoint start;
    Waypoint end;
    start.t = 0.1;
    start.pos.x = 0;
    start.pos.y = 0;
    start.pos.theta = 0;

    end.t = goalTime;
    end.pos = p;

    return createFromWaypoints({start, end}, isBaseFrame);
  }

  static CartesianGoal createFromWaypoints(std::vector<Waypoint> waypoints, bool isBaseFrame) {
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

  // Creates from cartesian trajectory.  May be (x,y) or (x,y,theta), but 
  // (x,y,theta) less likely to be realizable depending on system.
  // If "theta" is not defined, is tangent to path.
  // (x, y path here could be used for planning path through obstacles, for example)
  // \param isBaseFrame is "true" if this is relative to current body frame,
  // and "false" for using odometry coordinates.
  // TODO
  static CartesianGoal createFromTrajectory(CartesianTrajectory t, bool isBaseFrame);

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

  // TODO: some info here capturing above states/options
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

  MobileBase(Params p) :
    group_manager_(GroupManager::create(p))
  {}

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

      VectorXd compliantState;
      compliantState.resize(size);
      double nan = std::numeric_limits<double>::quiet_NaN();
      for (auto i=0; i<size; ++i) {
        compliantState(i) = nan;
      }

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

  //virtual SE2Point wheelsToSE2(Eigen::VectorXd wheel_state) const = 0;
  virtual Eigen::VectorXd SE2ToWheelVel(Pose pos, Vel vel) const = 0;

  // Get the global odometry state
  Pose getOdometry() const { return global_pose_ + relative_odom_offset_; };

  // Reset the current odometry state
  void setOdometry(Pose p) {
    last_wheel_pos_ = group_manager_->lastFeedback().getPosition();
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
    if (!group_manager_) {
      std::cout << "WARNING: Actuator Controller was not created successfully,"
	        << " cannot track goal!" << std::endl;
      return false;
    }

    auto baseTrajectories = buildTrajectory(g);
    if (baseTrajectories) {
      base_trajectories_ = *baseTrajectories;
      return true;
    }
    return false;
  }

  // When cleared, the base should be passive / compliant if possible.
  void clearGoal() {
    // Taken from https://stackoverflow.com/a/709161
    // clear the trajectory queue
    std::queue<shared_ptr<Trajectory>>().swap(base_trajectories_);
  }

  bool goalComplete() {
    if (base_trajectories_.empty())
      return true;
    if (base_trajectories_.back()->getEndTime() < group_manager_->lastTime())
      return true;
    return false;
  };

protected:
  // A cartesian trajectory is the only thing an individual base
  // implementation must handle.  This should be smoothly moved
  // to from the current state of the system.
  // TODO: maybe a std::vector<Trajectory>?
  // When "nullopt" is set, this clears t  // Creates from cartesian trajectory.  May be (x,y) or (x,y,theta), but 
  // (x,y,theta) less likely to be realizable depending on system.
  // If "theta" is not defined, is tangent to path.
  // (x, y path here could be used for planning path through obstacles, for example)
  // \param isBaseFrame is "true" if this is relative to current body frame,
  // and "false" for using odometry coordinates.he active trajectory
  // goal.
  // TODO: think about how to handle starting state to initial
  // state of trajectory -- should we just assume first waypoint
  // of trajectory is not at time 0, and add first point based on
  // current command (or feedback if not active) at time 0?)
  virtual std::unique_ptr<queue<shared_ptr<Trajectory>>> buildTrajectory(const CartesianGoal& g) = 0;

  virtual void updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) = 0;

  std::unique_ptr<GroupManager> group_manager_={nullptr};

  // These variables should be updated when updateOdometry is called
  Pose global_pose_{0, 0, 0};
  Vel global_vel_{0, 0, 0};
  VectorXd last_wheel_pos_{};

  Vel local_vel_{0, 0, 0};

  queue<shared_ptr<Trajectory>> base_trajectories_{};

private:
  Pose relative_odom_offset_{0, 0, 0};
};


} // namespace mobile
} // namespace experimental
} // namespace hebi
