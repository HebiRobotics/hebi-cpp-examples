#pragma once

#include "mobile_base.hpp"

namespace hebi {
namespace experimental {
namespace mobile {


class OmniBase : public MobileBase {
public:
  // Parameters for creating a base
  struct Params: public MobileBase::Params {
    double wheel_radius = 0.0762; // m
    double base_radius = 0.220; // m (center of omni to origin of base) 
  };

  static std::unique_ptr<OmniBase> create(Params p) {
    auto gm = GroupManager::create(p);
    if (!gm)
      return nullptr;

    auto retval = std::unique_ptr<OmniBase>(new OmniBase(move(gm), p));
    return retval;
  }

  virtual SE2Point wheelsToSE2(const Pose& pos,const Eigen::VectorXd&) const override;
  virtual Eigen::VectorXd SE2ToWheelVel(const Pose& pos, const Vel& vel) const override;
  virtual Vel getMaxVelocity() const override;

protected:
  // protected constructor
  OmniBase(std::unique_ptr<GroupManager> gm, Params p) : MobileBase(move(gm), p),
                       wheel_radius_(p.wheel_radius),
                       base_radius_(p.base_radius)
  {}

  // Updates local velocity based on wheel change in position since last time
  virtual void updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) override;

  // TODO: think about limitations on (x,y) vs. (x,y,theta)
  // trajectories
  std::unique_ptr<TrajectoryQueue> buildTrajectory(const CartesianGoal& g) override;

private:

  Eigen::Matrix3d thetaToTF(double theta) const {
    Eigen::Matrix3d tf = Matrix3d::Identity();
    tf.block<2,2>(0,0) = Rotation2D<double>(theta).toRotationMatrix();
    return tf;
  }

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

SE2Point OmniBase::wheelsToSE2(const Pose& pos, const Eigen::VectorXd& wheel_vels) const {
  auto local_base_vels = jacobian_inv_ * wheel_vels;
  // if provided, rotate around theta
  if (std::isnan(pos.theta))
    return {local_base_vels(0), local_base_vels(1), local_base_vels(2)};

  auto base_vels = thetaToTF(pos.theta) * local_base_vels;
  return {base_vels(0), base_vels(1), base_vels(2)};
}

Eigen::VectorXd OmniBase::SE2ToWheelVel(const Pose& pos, const Vel& vel) const {
  Eigen::VectorXd local_vel(3);
  local_vel(0) = vel.x;
  local_vel(1) = vel.y;
  local_vel(2) = vel.theta;

  // if position isn't provided, just use the local frame velocity
  if(std::isnan(pos.theta)) {
    return jacobian_ * local_vel;
  }

  Eigen::Vector3d global_vel = thetaToTF(pos.theta) * local_vel;

  return jacobian_ * global_vel;
};

Vel OmniBase::getMaxVelocity() const {
    // TODO: Do something smarter
    return Vel{2, 2, 2};
};

void OmniBase::updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) { 
  // Get local velocities
  auto local_vel = jacobian_inv_ * wheel_vel;
  local_vel_ = {local_vel(0), local_vel(1), local_vel(2)};

  // Get global velocity:
  auto global_vel = thetaToTF(global_pose_.theta) * local_vel;
  global_vel_ = {global_vel(0), global_vel(1), global_vel(2)};

  global_pose_ += global_vel_ * dt;
};

std::unique_ptr<TrajectoryQueue> OmniBase::buildTrajectory(const CartesianGoal& g) {
  // construct first waypoint from current state
  // so trajectory smoothly transitions

  // remove stale trajectories
  auto t_now = group_manager_->lastTime();
  while (!base_trajectories_.empty() && base_trajectories_.front()->getEndTime() < t_now) {
    base_trajectories_.pop();
  }

  Vector3d starting_vel;

  if (base_trajectories_.empty())
  {
    // if no trajectory, use last feedback.
    auto v = group_manager_->pendingCommand().getVelocity();
    auto base_vel = wheelsToSE2({}, v);
    starting_vel = {base_vel.x, base_vel.y, base_vel.theta};
  }
  else
  {
    auto traj = base_trajectories_.front();
    Eigen::VectorXd pos(3);
    Eigen::VectorXd vel(3);
    Eigen::VectorXd acc(3);
    traj->getState(t_now, &pos, &vel, &acc);
    starting_vel = thetaToTF(pos(2)) * vel;
  }

  auto zeros = VectorXd::Constant(3, 0.0);
  auto num_waypoints = g.times().size();
  VectorXd times(num_waypoints + 1);
  times.tail(num_waypoints) = g.times();
  times(0) = 0.0;

  MatrixXd pos(3, num_waypoints + 1);
  pos.col(0) = zeros;
  pos.rightCols(num_waypoints) = g.positions();

  MatrixXd vel(3, num_waypoints + 1);
  vel.col(0) = starting_vel;
  vel.rightCols(num_waypoints) = g.velocities();

  MatrixXd acc(3, num_waypoints + 1);
  acc.col(0) = zeros;
  acc.rightCols(num_waypoints) = g.accelerations();

  auto traj = Trajectory::createUnconstrainedQp(times,
                                                pos,
                                                &vel,
                                                &acc);

  if (!traj) {
    return nullptr;
  }

  std::unique_ptr<TrajectoryQueue> retval(new TrajectoryQueue());
  retval->push(traj);
  return retval;
};

} // namespace mobile
} // namespace experimental
} // namespace hebi
