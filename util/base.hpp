#pragma once

#include <optional>

#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"

#include "Eigen/Dense"

#include "group_trajectory_follower.hpp"

namespace hebi {
namespace experimental {
namespace mobile {

class SE2Point {
  public:
  double x{};
  double y{};
  double theta{};

  SE2Point operator+(const SE2Point& rhs) {
    SE2Point out;
    out.x = this->x + rhs.x;
    out.y = this->y + rhs.y;
    out.theta = this->theta + rhs.theta;
    return out;
  }

  SE2Point operator-(const SE2Point& rhs) {
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

typedef SE2Point Pose;
typedef SE2Point Vel;

Pose operator*(const Vel& val, double t) {
  SE2Point out{val.x*t, val.y*t, val.theta*t};
  return out;
}

Pose operator+(const Pose& v1, const Pose& v2) {
  SE2Point out{ v1.x + v2.x, v1.y + v2.y, v1.theta + v2.theta };
  return out;
}

class Waypoint {
  public:
  double t{};
  Pose pos{};
  Vel vel{};
  SE2Point acc{};
};

class CartesianTrajectory;

struct CartesianGoal {
  // Moves with the given relative body velocity for a certain
  // number of seconds, then slows to a stop over a certain number
  // of seconds.  Can be used for continuous replanning, e.g.
  // mobile IO con#include "kinematics_helper.hpp"trol.
  // \param v target velocity
  // \param cruiseTime time to drive at velocity v
  // \param slowTime "ramp down" time
  static CartesianGoal createFromVelocity(Vel v, double cruiseTime, double slowTime) {
    Waypoint start;
    start.t = 0;
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
    start.t = 0;
    start.pos.x = 0;
    start.pos.y = 0;
    start.pos.theta = 0;

    end.t = goalTime;
    end.pos = p;

    createFromWaypoints({start, end}, isBaseFrame);
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
  const Eigen::Matrix<double, 3, Eigen::Dynamic> positions_{0};
  const Eigen::Matrix<double, 3, Eigen::Dynamic> velocities_{0};
  const Eigen::Matrix<double, 3, Eigen::Dynamic> accelerations_{0};

  bool local_trajectory_{};
};

//////////////////////////////

class MobileBase {

public:

  // Parameters for creating a base
  struct Params: public GroupTrajectoryFollower::Params {
  };

  MobileBase(Params p) :
    trajectory_follower_(GroupTrajectoryFollower::create(p))
  {}

  bool update() {
    auto ret = trajectory_follower_->update();
    // now update odometry
    auto vel = trajectory_follower_->lastFeedback().getVelocity();
    updateOdometry(vel, trajectory_follower_->dT());
    return ret;
  }

  bool send() {
    return trajectory_follower_->send();
  }

  //virtual SE2Point wheelsToSE2(Eigen::VectorXd wheel_state) const = 0;
  virtual Eigen::VectorXd SE2ToWheelVel(Pose pos, Vel vel) const = 0;

  // Get the global odometry state
  Pose getOdometry() const { return global_pose_ + relative_odom_offset_; };

  // Reset the current odometry state
  void setOdometry(Pose p) { relative_odom_offset_ = p - getOdometry(); }

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
  // NOTE/TODO: alternatively, could pass a "Base" into the
  // Goal factory methods to convert to trajectory goal there.
  // Cleaner from the organization of the "Goal" class, but
  // messier otherwise -- e.g., user passing base into a function
  // that they are about to pass into base:
  // base.setGoal(Goal.createFrom(blah, base));
  // TODO: "best effort" -- or return "can't do this"?
  bool setGoal(const CartesianGoal& g) {
    auto jointsGoal = buildTrajectory(g);
    if (jointsGoal.has_value()) {
      trajectory_follower_->setGoal(jointsGoal.value());
      return true;
    }
    return false;
  }

  // When cleared, the base should be passive / compliant if possible.
  void clearGoal();

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
  virtual std::optional<Goal> buildTrajectory(const CartesianGoal& g) = 0;

  virtual void updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) = 0;

  std::unique_ptr<GroupTrajectoryFollower> trajectory_follower_={nullptr};

  // These variables should be updated when updateOdometry is called
  Pose global_pose_{0, 0, 0};
  Vel global_vel_{0, 0, 0};

  Vel local_vel_{0, 0, 0};

private:
  Pose relative_odom_offset_{0, 0, 0};
};


} // namespace mobile
} // namespace experimental
} // namespace hebi