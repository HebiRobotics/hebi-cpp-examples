#pragma once
#include "robot_model.hpp"
#include "Eigen/Dense"

/**
 * A helper function to get the torques which approximately balance out the
 * effect of gravity on the arm.
 */
Eigen::VectorXd getGravCompEfforts(
  const hebi::robot_model::RobotModel& model,
  const Eigen::VectorXd& masses,
  const Eigen::VectorXd& angles,
  const Eigen::Vector3d& gravity)
{
  // Normalize gravity vector (to 1g, or 9.8 m/s^2)
  Eigen::Vector3d normed_gravity = gravity;
  normed_gravity /= normed_gravity.norm();
  normed_gravity *= 9.81;

  int num_dof = model.getDoFCount();
  int num_frames = model.getFrameCount(HebiFrameTypeCenterOfMass);

  hebi::robot_model::MatrixXdVector jacobians;
  model.getJ(HebiFrameTypeCenterOfMass, angles, jacobians);

  // Get torque for each module
  // comp_torque = J' * wrench_vector
  // (for each frame, sum this quantity)
  Eigen::VectorXd comp_torque(num_dof);
  comp_torque.setZero();

  // Wrench vector
  Eigen::VectorXd wrench_vec(6); // For a single frame; this is (Fx/y/z, tau x/y/z)
  wrench_vec.setZero();
  for (int i = 0; i < num_frames; ++i)
  {
    // Set translational part
    for (int j = 0; j < 3; ++j)
    {
      wrench_vec[j] = -normed_gravity[j] * masses[i];
    }

    // Add the torques for each joint to support the mass at this frame
    comp_torque += jacobians[i].transpose() * wrench_vec;
  }

  return comp_torque;
}
