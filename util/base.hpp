#pragma once

#include <optional>

#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"

#include "Eigen/Dense"

namespace hebi {
namespace experimental {
namespace mobile {

class SE2Point {
  public:
  float x{};
  float y{};
  float theta{};

  SE2Point operator+(const SE2Point& rhs) {
    SE2Point out;
    out.x = this->x + rhs.x;
    out.y = this->y + rhs.y;
    out.theta = this->theta + rhs.theta;
  }

  SE2Point operator-(const SE2Point& rhs) {
    SE2Point out;
    out.x = this->x - rhs.x;
    out.y = this->y - rhs.y;
    out.theta = this->theta - rhs.theta;
  }
};

typedef SE2Point Pose;
typedef SE2Point Vel;

class Waypoint {
  public:
  double t{};
  Pose pos{};
  Vel vel{};
  SE2Point eff{};
};

// Potentially HEBI trajectory?  Maybe piecewise combination?
// (for velocity, can this be used )
class Trajectory;

struct Goal {
  // Moves with the given relative body velocity for a certain
  // number of seconds, then slows to a stop over a certain number
  // of seconds.  Can be used for continuous replanning, e.g.
  // mobile IO control.
  // \param v target velocity
  // \param cruiseTime time to drive at velocity v
  // \param slowTime "ramp down" time
  static Goal createFromVelocity(Vel v, double cruiseTime, double slowTime) {
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
  static Goal createFromPosition(Pose p, double goalTime, bool isBaseFrame) {
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

  // Creates from cartesian trajectory.  May be (x,y) or (x,y,theta), but 
  // (x,y,theta) less likely to be realizable depending on system.
  // If "theta" is not defined, is tangent to path.
  // (x, y path here could be used for planning path through obstacles, for example)
  // \param isBaseFrame is "true" if this is relative to current body frame,
  // and "false" for using odometry coordinates.
  static Goal createFromTrajectory(Trajectory t, bool isBaseFrame);
  static Goal createFromWaypoints(std::vector<Waypoint> waypoints, bool isBaseFrame);

  const Eigen::VectorXd& times() const { return times_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& positions() const { return positions_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& velocities() const { return velocities_; }
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& accelerations() const { return accelerations_; }

private:
  Goal(const Eigen::VectorXd& times,
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

class Base {

public:

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


  void update(float some_timestamp);
  // TODO: add a "send" and "getFeedback" and "getCommand" here?

  // Get the global odometry state
  virtual Pose getOdometry() const = 0;

  // Reset the current odometry state
  void setOdometry(Pose p) { _relative_odom_offset = p - getOdometry(); }

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
  void setGoal(const Goal& g);

  // When cleared, the base should be passive / compliant if possible.
  void clearGoal() { setTrajectory(std::nullopt); }

  // How far through the goal trajectory are we?  Returns "nullopt"
  // if there is no active goal.  Returns "1" if goal is complete
  // (base continues to command end state of trajectory if goal is
  // as "1", until goal is explicitly cleared)
  // TODO: might be able to move this to the parent class if additional
  // trajectory state was here, but I would think that might be
  // premature and limit the implementation of various Base types
  // for the purpose of saving a couple lines of boilerplate code.
  virtual std::optional<float> getGoalProgress() const = 0;

protected:
  // A cartesian trajectory is the only thing an individual base
  // implementation must handle.  This should be smoothly moved
  // to from the current state of the system.
  // TODO: maybe a std::vector<Trajectory>?
  // When "nullopt" is set, this clears the active trajectory
  // goal.
  // TODO: think about how to handle starting state to initial
  // state of trajectory -- should we just assume first waypoint
  // of trajectory is not at time 0, and add first point based on
  // current command (or feedback if not active) at time 0?)
  virtual void setTrajectory(std::optional<Trajectory> t) = 0;

private:
  std::function<double()> get_current_time_s_;
  Pose _relative_odom_offset{};
};

//////////////////////////////

class MyBase : Base {
public:
  Pose getOdometry() const override;
  std::optional<float> getGoalProgress() const override;
  Vel getMaxVelocity() const override;
protected:  
  // TODO: think about limitations on (x,y) vs. (x,y,theta)
  // trajectories
  void setTrajectory(std::optional<Trajectory> t) override;
};

} // namespace mobile
} // namespace experimental
} // namespace hebi
