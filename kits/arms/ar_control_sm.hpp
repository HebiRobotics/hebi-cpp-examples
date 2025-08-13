#pragma once

// HEBI C++ API files:
#include "arm/arm.hpp"
#include "color.hpp"
#include "util/vector_utils.h"
#include <Eigen/Dense>


using namespace hebi;

enum class ArmControlState { STARTUP, HOMING, TELEOP, DISCONNECTED, EXIT };

// This struct holds the inputs from the mobile IO device for controlling the arm
struct ArmMobileIOInputs {
  hebi::Vector3f phone_pos;
  Eigen::Matrix3d phone_rot;
  double ar_scaling{ 1.0f };
  bool lock_toggle{ false };
  bool locked{ true };
  bool gripper_closed{ false };
  bool home{ false };

  // Optional constructor if you want to set values on creation
  ArmMobileIOInputs(
    hebi::Vector3f pos = hebi::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(),
    double scaling = 1.0f,
    bool lockToggle = false,
    bool isLocked = true,
    bool gripperClosed = false,
    bool isHome = false);
};

// This class manages the control of the arm and mobile IO device, handling transitions between states and sending commands to the arm
class ArmMobileIOControl
{
public:
  std::string namespace_{ "" };
  ArmControlState state_{ ArmControlState::STARTUP };
  std::shared_ptr<arm::Arm> arm_ = nullptr;
  std::shared_ptr<arm::Gripper> gripper_ = nullptr;
  Eigen::Vector3d arm_xyz_home_{ 0.5, 0.0, 0.0 };
  Eigen::Matrix3d arm_rot_home_;
  double homing_time_{ 5.0 };
  double traj_duration_{ 1.0 };
  Eigen::VectorXd arm_seed_ik_;
  Eigen::VectorXd arm_home_;

  Eigen::Vector3d xyz_scale_{ 1.0, 1.0, 1.0 };
  Eigen::Vector3d last_locked_xyz_;
  Eigen::Matrix3d last_locked_rot_;
  Eigen::VectorXd last_locked_seed_;

  bool locked_{ true };
  double mobile_last_fbk_t_;

  std::vector<std::function<void(ArmMobileIOControl&, ArmControlState)>> transition_handlers_;

  hebi::Vector3f phone_xyz_home_{ 0.0, 0.0, 0.0 };
  Eigen::Matrix3d phone_rot_home_;

  ArmMobileIOControl(
    std::shared_ptr<arm::Arm> arm,
    std::shared_ptr<arm::Gripper> gripper,
    double homing_time,
    double traj_duration,
    Eigen::Vector3d xyz_scale);

  bool running() const { return state_ != ArmControlState::EXIT; }
  void send() const;
  void home(double duration) const;
  void transition_to(double t_now, const ArmControlState& new_state);

	// Computes the arm goal based on the current mobile IO inputs and the last locked position and rotation
  arm::Goal compute_arm_goal(const ArmMobileIOInputs& arm_input);

	// Updates the arm state based on the current time, mobile IO inputs, and sends commands to the arm
  void update(double t_now, const ArmMobileIOInputs* arm_input);

  void setArmLedColor(Color color);
  void stop();
};