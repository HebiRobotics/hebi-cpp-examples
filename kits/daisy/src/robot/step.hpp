#pragma once

#include "trajectory.hpp"
#include <Eigen/Dense>

namespace hebi {

class Leg;

// Represents a single step being actively taken by a leg.
class Step
{
public:
  Step(double start_time, const Leg& leg); // NOTE: I don't like this circular dependency on Leg here...
  // TODO: don't call update from constructor?

  // Note: returns 'true' if complete
  bool update(double t);

  const Eigen::Vector3d& getTouchDown() const;
  void computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::VectorXd& accels) const;

  static constexpr float period_ = 0.7f; // seconds
  static constexpr float overshoot_ = 0.3f; // factor of step to overshoot
  static constexpr float height_ = 0.04f; // in meters
  // When creating trajectories, don't use waypoints that are too close together
  static constexpr float ignore_waypoint_threshold_ = 0.01; // 10 ms (in seconds)
  double getStartTime() const { return start_time_; }
private:

  struct Keyframe {
    Keyframe(): p({0, 0, 0}), v({0, 0, 0}), a({0, 0, 0}){
    }
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
  };

  void computeKeyframes();
  void recomputeKeyframes();

  const Leg& leg_;
  // TODO: read from XML -- the phase points of the leg
  static const int num_phase_pts_ = 3;
  const double phase_[num_phase_pts_] = {0.0, 0.5, 1};
  double start_time_;
  // Time to hit each waypoint
  std::vector<double> time_;
  std::vector<Keyframe> keyframes_;

  Eigen::Vector3d lift_off_vel_;

  std::shared_ptr<trajectory::Trajectory> trajectory_;
  
  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace hebi
