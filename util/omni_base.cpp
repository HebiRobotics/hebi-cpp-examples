#include "omni_base.hpp"

namespace hebi {
namespace experimental {
namespace mobile {

Eigen::VectorXd OmniBase::SE2ToWheelVel(Pose pos, Vel vel) const {
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

  for(auto i = 0; i < num_waypoints; ++i) {
    auto p = g.positions().col(i);
    auto v = g.velocities().col(i);
    wheel_vels.col(i) = SE2ToWheelVel(Pose{p(0), p(1), p(2)}, Vel{v(0), v(1), v(2)});
  }

  return Goal::createFromWaypoints(g.times(),
                                   nan(3, num_waypoints),
                                   wheel_vels,
                                   nanWithZeroRight(3, num_waypoints));
};

} // namespace mobile
} // namespace experimental
} // namespace hebi