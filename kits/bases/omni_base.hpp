#pragma once

#include "group.hpp"
#include "group_command.hpp"
#include "trajectory.hpp"
#include "color.hpp"
#include <Eigen/Dense>


using namespace hebi;


// This struct represents the velocity of the chassis in 2D space with rotation around the Z-axis
struct ChassisVelocity {
  double x_;
  double y_;
  double rz_;
};

struct OmniBase {

  // Create omnibase and initialize the command
  OmniBase(std::shared_ptr<Group> group);
  bool setGains();

  // Evaluate Trajectory State and update commands
  void update(double t, double x_vel, double y_vel, double rot_vel);
  void buildVelocityTrajectory(double x_vel, double y_vel, double rot_vel, double t);
  void send();
  void stop();

  std::shared_ptr<Group> group_;

private:

  const double WHEEL_RADIUS = .15 / 2.0; //meters
  const double BASE_RADIUS = 0.235; //meters

  double last_time_{};
  double traj_start_time_{};
  const int base_num_wheels_{ 3 };

  GroupCommand base_command_;
  Eigen::Vector2d omni_base_traj_time_;
  Eigen::MatrixXd velocities_;
  Eigen::MatrixXd accelerations_;
  Eigen::MatrixXd jerks_;
  std::shared_ptr<trajectory::Trajectory> trajectory_;
  Color color_;

  const double a1 = -60 * M_PI / 180;
  const double a2 = 60 * M_PI / 180;
  const double a3 = 180 * M_PI / 180;

  const double base_chassis_mass_ = 12.0;
  const double base_chassis_inertia_zz_ = .5 * base_chassis_mass_ * BASE_RADIUS * BASE_RADIUS;
  const Eigen::MatrixXd chassis_mass_matrix_
  { (Eigen::MatrixXd(3,3) << base_chassis_mass_,0,0, 0,base_chassis_mass_,0, 0,0,base_chassis_inertia_zz_).finished() };

  const Eigen::Matrix3d base_wheel_transform_
  { (Eigen::MatrixXd(3,3) <<
   sin(a1), cos(a1), -BASE_RADIUS,
   sin(a2), cos(a2), -BASE_RADIUS,
   sin(a3), cos(a3), -BASE_RADIUS).finished() };
  const Eigen::Matrix3d base_wheel_velocity_matrix_{ base_wheel_transform_ / WHEEL_RADIUS };
  const Eigen::Matrix3d base_wheel_effort_matrix_{ base_wheel_transform_ * WHEEL_RADIUS };

  const double base_max_lin_speed_ = 0.6;
  const double base_max_rot_speed_ = base_max_lin_speed_ / BASE_RADIUS;
};
