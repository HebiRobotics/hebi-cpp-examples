#pragma once

#include "input_manager.hpp"
#include <Eigen/Dense>
#include "group.hpp"
#include <memory>

namespace hebi {
namespace input {

// If the I/O board/app is not found, command vectors return 0, and button
// um toggle/quit states are unchanged by 'update'. Class is not
// re-entrant.
class InputManagerMobileIO : public InputManager
{
public:
  InputManagerMobileIO();
  virtual ~InputManagerMobileIO() noexcept = default;

  // Connect to an I/O board/app and start getting feedback.  Return "true" if
  // found. Clears any existing connection
  bool reset();

  // A debug command that can be used to print the current state of the joystick
  // variables that are stored by the class.
  void printState() const override;

  // Get the current translation velocity command
  Eigen::Vector3f getTranslationVelocityCmd() const override;

  // Get the current rotation velocity command
  Eigen::Vector3f getRotationVelocityCmd() const override;

  // Returns true if the quit button has ever been pushed
  bool getQuitButtonPushed() const override;

  // Gets the number of times the mode button has been toggled since the last
  // request. Resets this count after retrieving.
  int getAndResetModeToggleCount() override;

  // Is the joystick connected?
  // Return "true" if we are connected to an I/O board/app; false otherwise.
  bool isConnected() const override {
    return group_ ? true : false;
  }

private:

  float getVerticalVelocity() const;

  // The Mobile IO app that serves as a joystick
  std::shared_ptr<hebi::Group> group_;

  // Scale the joystick scale to motion of the robot in SI units (m/s, rad/s,
  // etc).
  static constexpr float xyz_scale_ = 0.175;
  static constexpr float rot_scale_ = 0.4;

  float left_horz_raw_ = 0; // Rotation
  float left_vert_raw_ = 0; // Chassis tilt

  float slider_1_raw_ = 0; // Height

  float right_horz_raw_ = 0; // Translation (l/r)
  float right_vert_raw_ = 0; // Translation (f/b)

  bool prev_mode_button_state_ = false; // Mode
  int num_mode_toggles_ = 0;            //

  bool has_quit_been_pushed_ = false; // Quit
};

} // namespace input
} // namespace hebi

