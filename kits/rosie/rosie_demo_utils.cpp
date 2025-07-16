// HEBI C++ API files:
#include "arm/arm.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"
#include "util/mobile_io.hpp"
#include "util/vector_utils.h"
#include <cmath>
#include <Eigen/Dense>
#include "group.hpp"
#include "color.hpp"
#include <thread>
#include <chrono>

// Common includes
#include <iostream>

using namespace hebi;


struct OmniBase {
public:
  static constexpr double WHEEL_RADIUS = 0.0762; // meters
  static constexpr double BASE_RADIUS = 0.220;   // meters

  OmniBase(hebi::Group& group) : 
    group_(group),
    base_command(group.size()),
    base_feedback(group.size()),
    color(0, 0, 0)
  {
    vels_base_to_wheel = buildJacobian(BASE_RADIUS, WHEEL_RADIUS);
    trajectory = nullptr;
  }

  bool update(double t_now) {
    if (!group_.getNextFeedback(base_feedback))
      return false;

    if (trajectory) {
      Eigen::VectorXd p, v, a;
      trajectory->getState(t_now, &p, &v, &a);

      double theta = p[2];

      // Rotation from world to local frame
      Eigen::Matrix3d world_to_local_rot = Eigen::Matrix3d::Zero();
      world_to_local_rot(0, 0) = std::cos(theta);
      world_to_local_rot(0, 1) = -std::sin(theta);
      world_to_local_rot(1, 0) = std::sin(theta);
      world_to_local_rot(1, 1) = std::cos(theta);
      world_to_local_rot(2, 2) = 1.0;

      Eigen::Vector3d v_local = world_to_local_rot * v;

      Eigen::Vector3d wheel_vels = vels_base_to_wheel * v_local;
      base_command.setVelocity(wheel_vels);
	  base_command[0].led().set(color);
    }

    return true;
  }

  void send() {
    group_.sendCommand(base_command);
  }

  void buildSmoothVelocityTrajectory(double dx, double dy, double dtheta, double t_now) {
    Eigen::Vector4d times;
    times << 0, 0.15, 0.9, 1.2;

    Eigen::Vector3d cmd_vels = Eigen::Vector3d::Zero();
    if (trajectory) {
      Eigen::VectorXd pos, vel, acc;
      trajectory->getState(t_now, &pos, &vel, &acc);
      cmd_vels = vel;
    }

    Eigen::Vector3d target_vel(dx, dy, dtheta);
    Eigen::MatrixXd velocities(3, 4);
    velocities.col(0) = cmd_vels;
    velocities.col(1) = target_vel;
    velocities.col(2) = target_vel;
    velocities.col(3) = Eigen::Vector3d::Zero();

    Eigen::Vector4d full_times = (times.array() + t_now).matrix();

    buildVelocityTrajectory(full_times, velocities);
  }

  void buildVelocityTrajectory(const Eigen::VectorXd& times, const Eigen::MatrixXd& velocities) {
    Eigen::MatrixXd p = Eigen::MatrixXd::Constant(3, 4, std::numeric_limits<double>::quiet_NaN());
    p.col(0) = Eigen::Vector3d::Zero(); // only the first position is defined

    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(3, 4);
    trajectory = trajectory::Trajectory::createUnconstrainedQp(times, p, &velocities, &a);

  }

  hebi::GroupCommand base_command;

private:
  hebi::Group& group_;
  hebi::GroupFeedback base_feedback;
  std::shared_ptr<trajectory::Trajectory> trajectory;
  hebi::Color color;
  Eigen::Matrix3d vels_base_to_wheel;

  Eigen::Matrix3d buildJacobian(double base_radius, double wheel_radius) {
    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();

    jacobian(0, 0) = -std::sqrt(3.0) / 2.0;
    jacobian(1, 0) = std::sqrt(3.0) / 2.0;
    jacobian(2, 0) = 0.0;
    jacobian(0, 1) = -0.5;
    jacobian(1, 1) = -0.5;
    jacobian(2, 1) = 1.0;

    jacobian.col(2) = Eigen::Vector3d::Constant(-base_radius);
    jacobian /= wheel_radius;

    return jacobian;
  }
};

OmniBase setupBase(hebi::Lookup& lookup, const std::string& base_family) {
  const std::vector<std::string> wheel_names = { "W1", "W2", "W3" };

  // Create base group
  auto wheel_group = lookup.getGroupFromNames({ base_family }, wheel_names);
  if (!wheel_group || wheel_group->size() != 3) {
    throw std::runtime_error("Could not find wheel modules: W1, W2, W3 in family '" + base_family + "'");
  }

  // Return OmniBase object created with this group
  return OmniBase(*wheel_group);
}

void setMobileIOInstructions(util::MobileIO& mobile_io, const std::string& message, const uint8_t r, const uint8_t g, const uint8_t b) {

  if(r|g|b)
    mobile_io.setLedColor(r, g, b, false);

  mobile_io.clearText(false);
  mobile_io.appendText(message, false);

  std::cout << message << std::endl;
}

std::pair<std::unique_ptr<arm::Arm>, std::unique_ptr<arm::Gripper>> setupArm(const std::unique_ptr<RobotConfig>& example_config, const Lookup& lookup)
{
  std::unique_ptr<arm::Gripper> gripper;
  std::unique_ptr<arm::Arm> arm;

  try {
    arm = arm::Arm::create(*example_config, lookup);
  }
  catch (const std::exception& e) {
    std::cerr << "Error creating arm: " << e.what();
  }

  bool has_gripper = false;
  const auto user_data = example_config->getUserData();

  if (user_data.hasBool("has_gripper"))
    has_gripper = user_data.getBool("has_gripper");

  if (arm && has_gripper)
  {
    const std::string family = example_config->getFamilies()[0];
    auto gripper_group = lookup.getGroupFromNames({ family }, { "gripperSpool" });

    int tries = 3;
    while (!gripper_group && tries > 0)
    {
      std::cerr << "Looking for gripper module " << family << "/gripperSpool ...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
      gripper_group = lookup.getGroupFromNames({ family }, { "gripperSpool" });
      --tries;
    }

    if (!gripper_group)
      return std::make_pair(std::move(arm), std::move(gripper));

    double gripper_open_effort = -5; 
    double gripper_close_effort = 1;

    if(user_data.hasFloatList("gripper_open_effort"))
      gripper_open_effort = user_data.getFloat("gripper_open_effort");

	if (user_data.hasFloatList("gripper_close_effort"))
      gripper_close_effort = user_data.getFloat("gripper_close_effort");

	gripper = arm::Gripper::create(gripper_group, gripper_open_effort, gripper_close_effort);

    const std::string gripper_gains_file = example_config->getGains("gripper");
	gripper->loadGains(gripper_gains_file);
    gripper->open();
  }

  return std::make_pair(std::move(arm), std::move(gripper));
}

int main() {
  return 0;
}
