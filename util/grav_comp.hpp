#pragma once

#include "robot_model.hpp"
#include "group_feedback.hpp"
#include "Eigen/Dense"

namespace hebi {
namespace util {

inline Eigen::VectorXd getGravityCompensationEfforts(const hebi::robot_model::RobotModel& model, 
  const Eigen::VectorXd& masses, const Eigen::VectorXd& positions, const Eigen::Vector3d& gravity) {

  // Normalize gravity vector (to 1g, or 9.8 m/s^2)
  Eigen::Vector3d normed_gravity = gravity;
  normed_gravity /= normed_gravity.norm();
  normed_gravity *= 9.81;

  size_t num_dof = model.getDoFCount();
  size_t num_frames = model.getFrameCount(robot_model::FrameType::CenterOfMass);

  hebi::robot_model::MatrixXdVector jacobians;
  model.getJ(robot_model::FrameType::CenterOfMass, positions, jacobians);

  // Get torque for each module
  // comp_torque = J' * wrench_vector
  // (for each frame, sum this quantity)
  Eigen::VectorXd comp_torque(num_dof);
  comp_torque.setZero();

  // Wrench vector
  Eigen::VectorXd wrench_vec(6); // For a single frame; this is (Fx/y/z, tau x/y/z)
  wrench_vec.setZero();
  for (size_t i = 0; i < num_frames; i++) {
    // Set translational part
    for (size_t j = 0; j < 3; j++) {
      wrench_vec[j] = -normed_gravity[j] * masses[i];
    }

    // Add the torques for each joint to support the mass at this frame
    comp_torque += jacobians[i].transpose() * wrench_vec;
  }

  return comp_torque;
}

inline Eigen::VectorXd getGravityCompensationEfforts(const hebi::robot_model::RobotModel& model,
  const Eigen::VectorXd& masses, const hebi::GroupFeedback& feedback) {
  // Update gravity from base module:
  auto base_accel = feedback[0].imu().accelerometer().get();
  Vector3d gravity(-base_accel.getX(), -base_accel.getY(), -base_accel.getZ());

  return getGravityCompensationEfforts(model, masses, feedback.getPosition(), gravity);
}  

} // namespace util
} // namespace hebi

