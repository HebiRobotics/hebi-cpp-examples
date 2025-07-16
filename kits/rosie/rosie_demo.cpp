// HEBI C++ API files:
#include "arm/arm.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"
#include "util/mobile_io.hpp"
#include "util/vector_utils.h"
#include "rosie_demo_utils.cpp"
#include <Eigen/Dense>

// Common includes
#include <iostream>

using namespace hebi;

double ar_scaling = 1.0;

class ArmMobileIOInputs {
public:
    hebi::Vector3f phone_pos;
    Eigen::Matrix3d phone_rot;
    double ar_scaling;
    bool lock_toggle;
    bool locked;
    bool gripper_closed;
    bool home;

    // Optional constructor if you want to set values on creation
    ArmMobileIOInputs(
        const hebi::Vector3f& pos = hebi::Vector3f(0.0f, 0.0f, 0.0f),
        const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(),
        double scaling = 1.0f,
        bool lockToggle = false,
        bool isLocked = true,
        bool gripperClosed = false,
        bool isHome = false
    )
        : phone_pos(pos),
        phone_rot(rot),
        ar_scaling(scaling),
        lock_toggle(lockToggle),
        locked(isLocked),
        gripper_closed(gripperClosed),
        home(isHome)
    { }
};


class ChassisVelocity {
public:
  float x_;
  float y_;
  float rz_;

  ChassisVelocity(float x = 0.0f, float y = 0.0f, float rz = 0.0f) : x_(x), y_(y), rz_(rz) {}

  std::string getInfo() const
  {
    std::string info = "ChassisVelocity : x=" + std::to_string(x_) + ", y=" + std::to_string(y_) + ", rz=" + std::to_string(rz_);
    return info;
  }
};

enum class ArmControlState { STARTUP, HOMING, TELEOP, DISCONNECTED, EXIT };

class RosieControl {
  public:
    RosieControl(std::shared_ptr<OmniBase> base)
        : base_(base), running_(true), on_shutdown_([](){}) {
    }

    void update(double t_now, std::shared_ptr<ChassisVelocity> base_inputs)
    {
        base_->update(t_now);
		if (!base_inputs)
          return;
		base_->buildSmoothVelocityTrajectory(base_inputs->x_, base_inputs->y_, base_inputs->rz_, t_now);
    }

    void send() {
        base_->send();
    }
    
    void stop() {
        running_ = false;
		base_->base_command.setVelocity(Eigen::Vector3d::Zero());
		on_shutdown_();
    }

  bool running_;
  std::function<void()> on_shutdown_;

private:
  std::shared_ptr<OmniBase> base_;
};

void setupMobileIO(util::MobileIO& m) {
  //Sets up mobileIO interface.
  // Return a function that parses mobileIO feedback into the format expected by the Demo

    const int reset_pose_btn = 1;
    const int arm_lock = 3;
    const int gripper_close = 4;
    const int quit_demo_btn = 8;
    
    const int side_joy = 1;  // Left Pad Left/Right
    const int forward_joy = 2; // Left Pad Up/Down
    const int ar_xyz_scale_slider = 4;
    const int turn_joy = 7;  // Right Pad Left/Right
	const int rotate_joy = 8; // Right Pad Up/Down

    m.resetUI();

    m.setButtonLabel(reset_pose_btn, "\u27F2", false);
    m.setButtonLabel(quit_demo_btn, "\u274C", false);

    m.setAxisLabel(side_joy, "", false);
    m.setAxisLabel(forward_joy, "Translate", false);
    m.setAxisLabel(ar_xyz_scale_slider, "XYZ\nScale", false);
    m.setAxisLabel(turn_joy, "", false);
    m.setAxisLabel(rotate_joy, "Rotate", false);

    for (int i = 0; i < 8; i++) {
      m.setAxisSnap(i + 1, (i == 3) ? NAN : 0.0);
    }

    m.setAxisValue(ar_xyz_scale_slider, ar_scaling);
    m.setLedColor(255, 255, 0);

    m.setButtonLabel(arm_lock, "Arm \U0001F512", false);
    m.setButtonLabel(gripper_close, "Gripper", false);
    m.setButtonMode(arm_lock, util::MobileIO::ButtonMode::Toggle);
    m.setButtonMode(gripper_close, util::MobileIO::ButtonMode::Toggle);

}

bool parseMobilIOFeedback(util::MobileIO& m, ChassisVelocity& chassis_velocity, ArmMobileIOInputs& arm_inputs ) {
    // ?? Timeout is 0 seconds??
      // Button/Axis mappings
    int reset_pose_btn = 1;
    int quit_demo_btn = 8;

    int side_joy = 1;
    int forward_joy = 2;
    int ar_xyz_scale_slider = 4;
    int turn_joy = 7;

    int arm_lock = 3;
    int gripper_close = 4;

    if (!m.update(0.0))
        return false;

    if (m.getButton(quit_demo_btn))
        return true;

    if (m.getButton(reset_pose_btn))
    {
        chassis_velocity = ChassisVelocity();
        //arm_inputs = ArmMobileIOInputs();
        return false;
    }

    chassis_velocity = ChassisVelocity(
        pow(m.getAxis(forward_joy), 3),
        pow(-m.getAxis(side_joy), 3),
        pow(-m.getAxis(turn_joy), 3) * 2);

  auto wxyz = m.getArOrientation();
  Eigen::Quaterniond q(wxyz.getW(), wxyz.getX(), wxyz.getY(), wxyz.getZ());
  Eigen::Matrix3d rotation = q.toRotationMatrix();

  if(!rotation.allFinite()) {
    std::cerr << "Error getting orientation as matrix: " << wxyz.getW() << ", " << wxyz.getX() << ", "
              << wxyz.getY() << ", " << wxyz.getZ() << "\n" ;
    rotation = Eigen::Matrix3d::Identity();
  }

  if (m.getButtonDiff(arm_lock) != util::MobileIO::ButtonState::Unchanged)
  {
      if(!m.getButton(arm_lock))
          m.setButtonLabel(arm_lock, "Arm \U0001F512", false);
      else
		  m.setButtonLabel(arm_lock, "Arm \U0001F513", false);
  }

  auto locked = m.getButton(arm_lock);
  if(locked) {
    m.setAxisValue(ar_xyz_scale_slider, (2 * ar_scaling) - 1.0);
  } else {
    ar_scaling = (m.getAxis(ar_xyz_scale_slider) + 1.0) / 2.0;
    if (ar_scaling < 0.1)
      ar_scaling = 0.0;
  }
 
  arm_inputs = ArmMobileIOInputs(
      m.getArPosition(),
      rotation,
      ar_scaling,
      (m.getButtonDiff(arm_lock) != util::MobileIO::ButtonState::Unchanged),
      !locked,
      m.getButton(gripper_close)
  );

  return false;
}