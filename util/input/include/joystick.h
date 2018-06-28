#pragma once

#include <SDL.h>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace hebi {
namespace util {

/**
 * Represents the value of a hat
 */
struct HatValue {
  int8_t x, y;
};

// Type alias for axis event callbacks
using AxisEventHandler = std::function<void(uint32_t, float)>;

// Type alias for hat event callbacks
using HatEventHandler = std::function<void(uint32_t, HatValue)>;

// Type alias for button event callbacks
using ButtonEventHandler = std::function<void(uint32_t, bool)>;

// For internal use only.
class JoystickImpl;

/**
 * Class which represents a controller or joystick
 */
class Joystick {

private:

  struct ctor_key {};

  friend class JoystickDispatcher;
  friend JoystickImpl;
  friend class JoystickMapper;

  mutable std::mutex lifecycle_lock_;
  mutable JoystickImpl* impl_;

  std::unique_lock<std::mutex> scoped_lock();
  bool is_disposed(const char* func) const;
  void dispose();

public:

//------------------------------------------------------------------------------

  // Do not use directly -- hence the `ctor_key`
  Joystick(JoystickImpl* impl, ctor_key)
    : impl_(impl) {}

  // Explicitly deleted to ensure only one constructor signature is allowed
  Joystick() = delete;
  Joystick(const Joystick&) = delete;
  Joystick(Joystick&&) = delete;

//------------------------------------------------------------------------------

  /**
   * Retrieve the number of joysticks found by the SDL2 interface. The value
   * returned from this function does not imply that there are as many
   * available Joystick instances. This function mainly serves as a wrapper
   * around the SDL2 function SDL_NumJoysticks.
   */
  static size_t joystick_count();

  /**
   * Retrieves the joystick at the given index.
   * 
   * A null pointer is returned when one of the conditions has been met:
   *   The index is greater than the number of joysticks
   *   The joystick at the given index is not a game controller
   *   The event handling system is not loaded (or has been destroyed)
   * 
   * @param index the index of the joystick to be retrieved. This corresponds
   *              to the index of the joystick/gamecontroller in the SDL2 API
   *              (e.g., SDL_JoystickOpen, SDL_GameControllerOpen)
   */
  static std::shared_ptr<Joystick> at_index(size_t index);

  /**
   * Retrieves a vector of all available joysticks at the given point in time
   */
  static std::vector<std::shared_ptr<Joystick>> available_joysticks();

//------------------------------------------------------------------------------

  /**
   * The human readable string of the button at the given index.
   * If the provided index is out of bounds, "Invalid" is returned.
   */
  std::string get_button_name(size_t button) const;
  
  /**
   * The human readable string of the axis at the given index.
   * If the provided index is out of bounds, "Invalid" is returned.
   */
  std::string get_axis_name(size_t axis) const;

  /**
   * The human readable name of the joystick.
   * This is also the string returned by SDL_GameControllerName
   */
  std::string name() const;

  /**
   * The GUID returned by SDL_JoystickGetGUID in string form
   */
  std::string guid() const;

  /**
   * The index of this joystick in the SDL api at the time the
   * event handler API was initialized.
   */
  size_t index() const;

//------------------------------------------------------------------------------

  /**
   * Adds a callback for the given axis. The provided callback is invoked
   * whenever movement is detected for the given axis.
   */
  void add_axis_event_handler(size_t axis, AxisEventHandler handler);

  /**
   * Adds a callback for the given axis. The provided callback is invoked
   * whenever movement is detected for the given axis.
   * 
   * If the string provided is not a valid axis name, an exception will be thrown.
   */
  void add_axis_event_handler(const std::string& axis, AxisEventHandler handler);

  /**
   * Adds a callback for the given button. The provided callback is invoked
   * whenever a press or release is detected for the given button.
   */
  void add_button_event_handler(size_t button, ButtonEventHandler handler);

  /**
   * Adds a callback for the given button. The provided callback is invoked
   * whenever a press or release is detected for the given button.
   * 
   * If the string provided is not a valid button name, an exception will be thrown.
   */
  void add_button_event_handler(const std::string& button, ButtonEventHandler handler);

//------------------------------------------------------------------------------

  /**
   * Retrieves the current value of the provided axis.
   * 
   * If the index is out of range (i.e., greater than/equal to the number of axes),
   * an exception is thrown.
   */
  float get_current_axis_state(size_t axis);

  /**
   * Retrieves the current value of the provided axis.
   * 
   * If the provided name does not represent an axis, an exception is thrown.
   */
  float get_current_axis_state(const std::string& axis);

  /**
   * Retrieves the current value of the provided button.
   * 
   * If the index is out of range (i.e., greater than/equal to the number of buttons),
   * an exception is thrown.
   */
  bool get_current_button_state(size_t button);

  /**
   * Retrieves the current value of the provided button.
   * 
   * If the provided name does not represent a button, an exception is thrown.
   */
  bool get_current_button_state(const std::string& button);

//------------------------------------------------------------------------------

  /**
   * Waits for the next detected movement of the axis and returns
   * the detected value.
   * 
   * If the index is out of range (i.e., greater than/equal to the number of axes),
   * an exception is thrown.
   */
  float get_next_axis_state(size_t axis);

  /**
   * Waits for the next detected press or release of the button and returns
   * the detected value.
   * 
   * If the index is out of range (i.e., greater than/equal to the number of buttons),
   * an exception is thrown.
   */
  bool get_next_button_state(size_t button);

};


}
}
