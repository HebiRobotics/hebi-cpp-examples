/*
 * Contains functions to set the mobile IO display for the double arm
 * teach repeat example.
 */

#include <set>

#include "util/mobile_io.hpp"

namespace hebi {
namespace examples {

constexpr double gripperEffort(arm::Gripper::State gs);

// Simple helper class for sending mobile IO state
struct MobileIoState {
public:
  // Sends the entire state to the mobile IO device, defaulting
  // to empty labels
  bool sendTo(util::MobileIO& mobile_io) const {
    bool success = true;
    success = mobile_io.clearText() && success;
    success = mobile_io.appendText(text_) && success;
    for (int i = 1; i <= 8; ++i) {
      success = mobile_io.setButtonLabel(i, button_labels_[i - 1]) && success;
      success = mobile_io.setAxisLabel(i, axis_labels_[i - 1]) && success;
      if (set_axis_enable_[i - 1])
        success = mobile_io.setAxisValue(i, set_axis_value_[i - 1]) && success;
      if (set_button_enable_[i - 1])
        success = mobile_io.setButtonLed(i, set_button_value_[i - 1]) && success;
    }
    success = mobile_io.setLedColor(led_color_.getRed(), led_color_.getGreen(), led_color_.getBlue()) && success;

    return success;
  }

  bool sendTo(util::MobileIO& mobile_io, int num_attempts) const {
    for (int i = 0; i < num_attempts; ++i)
    {
      if (sendTo(mobile_io))
        return true;
    }
    return false;
  }

  void setText(std::string text)
  {
    text_ = text;
  }

  std::string text()
  {
      return text_;
  }

  void setColor(hebi::Color color)
  {
    led_color_ = color;
  }

  void setButtonLabel(int number, std::string label)
  {
    if (number < 1 || number > util::MobileIO::NumButtons)
      return;
    button_labels_[number - 1] = label;
  }

  void setAxisLabel(int number, std::string label)
  {
    if (number < 1 || number > util::MobileIO::NumButtons)
      return;
    axis_labels_[number - 1] = label;
  }

  void setAxisValue(int number, float value)
  {
    if (number < 1 || number > util::MobileIO::NumButtons)
      return;
    set_axis_enable_[number - 1] = true;
    set_axis_value_[number - 1] = value;
  }

  void clearAxisValue(int number)
  {
    if (number < 1 || number > util::MobileIO::NumButtons)
      return;
    set_axis_enable_[number - 1] = false;
  }

  void setButtonLed(int number, bool value)
  {
    if (number < 1 || number > util::MobileIO::NumButtons)
      return;
    set_button_enable_[number - 1] = true;
    set_button_value_[number - 1] = value;
  }

  // Don't actually turn it off; just set it to not set either way
  // in the state.
  void clearButtonLed(int number)
  {
    if (number < 1 || number > util::MobileIO::NumButtons)
      return;
    set_button_enable_[number - 1] = false;
  }

private:
  std::string text_;
  hebi::Color led_color_;
  std::array<std::string, util::MobileIO::NumButtons> button_labels_;
  std::array<std::string, util::MobileIO::NumButtons> axis_labels_;
  // Note -- std::optional (C++17) would be preferable here
  // Which axis values to set
  std::array<bool, util::MobileIO::NumButtons> set_axis_enable_;
  // The value of those axes to set
  std::array<float, util::MobileIO::NumButtons> set_axis_value_; 
  // Which button values to set
  std::array<bool, util::MobileIO::NumButtons> set_button_enable_;
  // The value of those button to set
  std::array<bool, util::MobileIO::NumButtons> set_button_value_; 
};

MobileIoState trainingDisplay(bool repeat_mode)
{
  MobileIoState st;
  st.setText("STOP : Add waypoint (stop)\nGRIP : Add waypoint (grip)\n"
             "FLOW : Add waypoint (flow)\nCLR : Clear waypoints\n"
             "(waypoints are added for current position of both arms)\n"
             "\n"
             "Enter playback with \u25b6\n"
             "\n"
             "Select active arm, time between waypoints, and playback mode below\n");
  
  st.setColor({0,0,255});
  
  st.setButtonLabel(1, "STOP");
  st.setButtonLabel(2, "GRIP");
  st.setButtonLabel(3, "FLOW");
  st.setButtonLabel(4, "CLR");
  st.setButtonLabel(5, "\U0001F4BE");
  st.setButtonLabel(6, "\U0001F4C2");
  st.setButtonLabel(7, "\u25b6");
  st.setButtonLabel(8, "\u274c");

  // When coming back to training, we may be coming from "save" screen with highlighted buttons:
  for (int i = 1; i <= util::MobileIO::NumButtons; ++i)
    st.setButtonLed(i, 0);
  
  // Reset to "none" for arm select when returning 
  st.setAxisValue(3, 0);
  st.setAxisLabel(3, "None");
  st.setAxisLabel(4, "\u231b");
  st.setAxisLabel(5, repeat_mode ? "Repeat" : "Single");

  return st;
}

MobileIoState playbackDisplay()
{
  MobileIoState st;
  st.setText("Return to training with \u23f8\n");
  st.setColor({0,255,0});
  st.setButtonLabel(7, "\u23f8");
  st.setButtonLabel(8, "\u274c");

  return st;  
}
    
MobileIoState loadDisplay(const std::map<int, std::string>& button_file_map,
                          const std::set<std::string>& existing_files)
{
  MobileIoState st;
  st.setText("Select saved waypoints to load, or press BACK to return to training mode with no changes.\n");
  st.setColor({255,0,0});
  for (auto kvp : button_file_map)
  {
    if (existing_files.count(kvp.second) > 0)
      st.setButtonLabel(kvp.first, kvp.second);
  }
  st.setButtonLabel(6, "BACK"); 
  st.setButtonLabel(8, "\u274c");

  return st;
}

MobileIoState saveDisplay(const std::map<int, std::string>& button_file_map,
                          const std::set<std::string>& existing_files)
{
  MobileIoState st;
  st.setText("Select where to save waypoints, or return to previous screen with no changes.\n"
             "Highlighted options have existing waypoints that will be overwritten.\n");
  st.setColor({255,255,0});
  for (auto kvp : button_file_map)
  {
    st.setButtonLabel(kvp.first, kvp.second);
    if (existing_files.count(kvp.second) > 0)
      st.setButtonLed(kvp.first, 1);
    else
      st.setButtonLed(kvp.first, 0);
  }
  st.setButtonLabel(6, "BACK"); 
  st.setButtonLabel(8, "\u274c");

  return st;
}

} // namespace examples
} // namespace hebi