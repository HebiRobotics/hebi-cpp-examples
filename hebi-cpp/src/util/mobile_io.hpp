#pragma once

/**
 * \file mobile_io.hpp
 *
 * Contains utility class and data structures to assist with use of a mobile IO
 * application as a controller for a robotic system.
 */

#include <array>
#include <bitset>
#include <memory>

#include "group.hpp"
#include "group_feedback.hpp"

namespace hebi {
namespace experimental {

static constexpr size_t NumButtons = 8;

struct MobileIODiff;
class MobileIO;

// The current state at any time
struct MobileIOState {
  // Note: one-indexed to match buttons on the screen
  bool getButton(size_t button) const;
  float getAxis(size_t axis) const;

private:
  std::bitset<NumButtons> buttons_;
  std::array<float, NumButtons> axes_;

  friend struct MobileIODiff;
  friend class MobileIO;
};

// Difference between two IO states, useful for checking to see if a button 
// has been pressed.
struct MobileIODiff {
  MobileIODiff(const MobileIOState& prev, const MobileIOState& current);

  enum class ButtonState {
    Off, On, // These occur if last + current state are the same
    ToOff, ToOn // Edge triggers; these occur if last + current state are different
  };

  // Note: one-indexed to match buttons on the screen
  ButtonState get(int button) const;

private:
  std::array<ButtonState, NumButtons> buttons_;
};

// Wrapper around a mobile IO controller
class MobileIO {
public:
  enum class ButtonMode {
    Momentary, Toggle
  };  

  static std::unique_ptr<MobileIO> create(const std::string& family, const std::string& name);

  // Input/feedback
  MobileIOState getState();
  // Input/feedback; the "got_feedback" flag indicates if feedback was received within the timeout
  // or not.
  MobileIOState getState(bool& got_feedback);

  // Outputs
  // Note: one-indexed to match axes/buttons on the screen

  bool disableSnap(size_t axis_number) {
    return setSnap(axis_number, std::numeric_limits<float>::quiet_NaN());
  }

  bool setSnap(size_t axis_number, float snap_to);
  bool setAxisValue(size_t axis_number, float value);

  bool setButtonMode(size_t button_number, ButtonMode mode);
  bool setButtonOutput(size_t button_number, bool on);  

  bool setLedColor(uint8_t r, uint8_t g, uint8_t b);

  bool sendText(const std::string& message);
  bool clearText();

  // Return Feedback object specific to the mobile device (not GroupFeedback)
  const hebi::Feedback& getLastFeedback() const { return fbk_[0]; };

private:
  MobileIO(std::shared_ptr<hebi::Group>);

  std::shared_ptr<hebi::Group> group_;
  hebi::GroupFeedback fbk_;
  MobileIOState current_state_;
};

} // namespace experimental
} // namespace hebi
