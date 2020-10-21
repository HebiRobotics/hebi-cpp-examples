#include "input_manager_mobile_io.hpp"
#include "lookup.hpp"
#include "xml_util/pugixml.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"

#include <iostream> // Note: for debugging output.

namespace hebi {
namespace input {

InputManagerMobileIO::InputManagerMobileIO()
{
  if (!reset())
    std::cout << "Could not find joystick input board!" << std::endl;
}

bool InputManagerMobileIO::reset()
{
  // Cleanup if we already have one!
  if (group_)
    group_.reset();

  // Look for mobile IO
  hebi::Lookup lookup;
  long timeout_ms = 2000; // use a 2 second timeout
  group_ = lookup.getGroupFromNames({"Daisy"}, {"mobileIO"}, timeout_ms);
  if (!group_)
    return false;

  // Set Slider snap positions and button toggle modes.
  GroupCommand cmd(1);
  cmd[0].io().a().setFloat(3, 0);
  cmd[0].io().f().setFloat(3, 0);
  cmd[0].io().b().setInt(7, 1);
  group_->sendCommandWithAcknowledgement(cmd);

  group_->addFeedbackHandler([this] (const GroupFeedback& fbk)
  {
    // TODO: use 'std::atomic' variables for state here? Add mutex?  Does this
    // slow things down due to lock contention?
    const auto& analog = fbk[0].io().a();
    const auto& digital = fbk[0].io().b();
    if (analog.hasFloat(1) && analog.hasFloat(2) && analog.hasFloat(3) &&
        digital.hasInt(1))
    {
      left_horz_raw_ = analog.getFloat(1);
      left_vert_raw_ = analog.getFloat(2);

      slider_1_raw_ = analog.getFloat(3);

      right_horz_raw_ = analog.getFloat(7);
      right_vert_raw_ = analog.getFloat(8);

      // Note: Button is configured at toggle, so don't need to count edges
      mode_ = digital.getInt(7);

      // Check for "Quit"
      if (digital.getInt(8) == 1) {
        has_quit_been_pushed_ = true;
      }
    }
  });
  group_->setFeedbackFrequencyHz(1000); // Don't want to miss button presses!
  return true;
}

void InputManagerMobileIO::printState() const
{
  std::cout << "Rotation (z)" << rot_scale_ * left_horz_raw_ << "\n";
  std::cout << "Rotation (y)" << rot_scale_ * left_vert_raw_ << "\n";

  std::cout << "Translation (z)" << xyz_scale_ * left_horz_raw_ << "\n";
  std::cout << "Translation (y)" << xyz_scale_ * right_vert_raw_ << "\n";

  std::cout << "Height " << xyz_scale_ * getVerticalVelocity() << "\n";

  std::cout << "quit state: " << has_quit_been_pushed_ << "\n";
  std::cout << "Mode: " << mode_ << "\n";
}

Eigen::Vector3f InputManagerMobileIO::getTranslationVelocityCmd() const
{
  Eigen::Vector3f translation_velocity_cmd;
  if (isConnected())
  {
    translation_velocity_cmd <<
      -xyz_scale_ * right_vert_raw_,
      xyz_scale_ * right_horz_raw_,
      xyz_scale_ * getVerticalVelocity();
  }
  else
  {
    translation_velocity_cmd.setZero();
  }
  return translation_velocity_cmd;
}

Eigen::Vector3f InputManagerMobileIO::getRotationVelocityCmd() const
{
  Eigen::Vector3f rotation_velocity_cmd;
  if (isConnected())
  {
    rotation_velocity_cmd <<
      0,
      -rot_scale_ * left_vert_raw_,
      rot_scale_ * left_horz_raw_;
  }
  else
  {
    rotation_velocity_cmd.setZero();
  }
  return rotation_velocity_cmd;
}

bool InputManagerMobileIO::getQuitButtonPushed() const
{
  return has_quit_been_pushed_;
}

size_t InputManagerMobileIO::getMode()
{
  return mode_;
}
  
float InputManagerMobileIO::getVerticalVelocity() const
{
  // Slider dead zone: 0.25
  const float slider_dead_zone_ = 0.25; 
  if (std::abs(slider_1_raw_) < slider_dead_zone_)
    return 0;
  // Scale from [dead_zone, 1] to [0, 1] (and same for [-1, -dead_zone]
  if (slider_1_raw_ > 0)
    return -(slider_1_raw_ - slider_dead_zone_) / (1 - slider_dead_zone_);
  return -(slider_1_raw_ + slider_dead_zone_) / (1 - slider_dead_zone_);
}

} // namespace input
} // namespace hebi
