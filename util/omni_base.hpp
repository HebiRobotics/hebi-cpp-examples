#pragma once

#include "base.hpp"

namespace hebi {
namespace experimental {
namespace mobile {

class OmniBase : public MobileBase {
public:
  // Parameters for creating a base
  struct Params: public MobileBase::Params {
    Params() : wheel_radius(0.0762), base_radius(0.220) {}
    double wheel_radius; // m
    double base_radius; // m (center of omni to origin of base) 
  };

  OmniBase(Params p) : MobileBase(p),
                       wheel_radius_(p.wheel_radius),
                       base_radius_(p.base_radius)
  {}

  virtual Eigen::VectorXd SE2ToWheelVel(Pose pos, Vel vel) const override;
  virtual Vel getMaxVelocity() const override;

protected:
  // Updates local velocity based on wheel change in position since last time
  virtual void updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) override;

private:

  Eigen::Matrix3d createJacobian() {
    double a = sqrt(3)/(2 * wheel_radius_);
    double b = 1.0 / wheel_radius_;
    double c = -base_radius_ / wheel_radius_;
    Eigen::Matrix3d j;
    j << -a, -b / 2.0, c,
         a, -b / 2.0, c,
         0.0, b, c;
    return j;
  }

  Eigen::Matrix3d createJacobianInv() {
    Eigen::Matrix3d j_inv;
    double a = wheel_radius_ / sqrt(3);
    double b = wheel_radius_ / 3.0;
    double c = -b / base_radius_;
    j_inv << -a, a, 0,
             -b, -b, 2.0 * b,
             c, c, c;
    return j_inv;
  }

  // TODO: think about limitations on (x,y) vs. (x,y,theta)
  // trajectories
  virtual std::optional<Goal> buildTrajectory(const CartesianGoal& g) override;

  // Helper function to create unconstrained points along a motion.
  static MatrixXd nan(size_t num_joints, size_t num_waypoints) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    return matrix;
  }

  // Helper function to create unconstrained points along a motion, with nan at the right side.
  static MatrixXd nanWithZeroRight(size_t num_joints, size_t num_waypoints) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    matrix.rightCols<1>().setZero();
    return matrix;
  }

   /* Declare main kinematic variables */
  double wheel_radius_; // m
  double base_radius_; // m (center of omni to origin of base) 

  const Eigen::Matrix3d jacobian_ = createJacobian();

  // Wheel velocities to local (x,y,theta)
  const Eigen::Matrix3d jacobian_inv_ = createJacobianInv();
};


Eigen::VectorXd OmniBase::SE2ToWheelVel(Pose pos, Vel vel) const {
  // if position isn't provided, just return the local frame velocity
  if(std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.theta)) {
    Eigen::VectorXd ret(3);
    ret(0) = vel.x;
    ret(1) = vel.y;
    ret(2) = vel.theta;
    return jacobian_ * ret;
  }

  double theta = pos.theta;
  double dtheta = vel.theta;

  double offset = 1.0;
  double ctheta = std::cos(-theta);
  double stheta = std::sin(-theta);
  double dx = vel.x * ctheta - vel.y * stheta;
  double dy = vel.x * stheta + vel.y * ctheta;

  Eigen::Vector3d local_vel;
  local_vel << dx, dy, dtheta;

  return jacobian_ * local_vel;
};

Vel OmniBase::getMaxVelocity() const {
    // TODO: Do something smarter
    return Vel{2, 2, 2};
};

void OmniBase::updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) { 
  // Get local velocities
  auto local_vel = jacobian_inv_ * wheel_vel;
  local_vel_.x = local_vel[0];
  local_vel_.y = local_vel[1];
  local_vel_.theta = local_vel[2];

  // Get global velocity:
  auto c = std::cos(global_pose_.theta);
  auto s = std::sin(global_pose_.theta);
  global_vel_.x = c * local_vel_.x - s * local_vel_.y;
  global_vel_.y = s * local_vel_.x + c * local_vel_.y;
  // Theta transforms directly
  global_vel_.theta = local_vel_.theta;

  global_pose_ += global_vel_ * dt;
};

std::optional<Goal> OmniBase::buildTrajectory(const CartesianGoal& g) {
  auto num_waypoints = g.times().size();
  MatrixXd wheel_vels(3, num_waypoints);

  // TODO: REDO THIS PROPERLY IT SUCKS/IS BROKEN
  // need to have some kind of non-hacky 2d trajectory rollout,
  // or drop this approach entirely

  std::cout << "Build Trajectory" << std::endl;

  for(auto i = 0; i < num_waypoints; ++i) {
    Vector3d p = g.positions().col(i);
    Vector3d v = g.velocities().col(i);
    if(std::isnan(v(0)) || std::isnan(v(1)) || std::isnan(v(2))) {
      if(i == num_waypoints - 1) {
        v(0) = 0;
        v(1) = 0;
        v(2) = 0;
      } else {
	auto delta = p - g.positions().col(i+1);
	auto dt = g.times()(i+1) - g.times()(i);
        v(0) = delta(0)/dt;
        v(1) = delta(1)/dt;
        v(2) = delta(2)/dt;
      }
    }
    std::cout << "P:\n" << p << std::endl;
    std::cout << "V:\n" << v <<std::endl;
    wheel_vels.col(i) = SE2ToWheelVel(Pose{p(0), p(1), p(2)}, Vel{v(0), v(1), v(2)});
  }

  std::cout << "Times:\n" << g.times() << std::endl;
  std::cout << "Vels:\n" << wheel_vels << std::endl;

  return Goal::createFromWaypoints(g.times(),
                                   nan(3, num_waypoints),
                                   wheel_vels,
                                   nanWithZeroRight(3, num_waypoints));
};

} // namespace mobile
} // namespace experimental
} // namespace hebi
