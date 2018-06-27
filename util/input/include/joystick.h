#pragma once

#include <SDL.h>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace hebi {
namespace util {

struct HatValue {
  int8_t x, y;
};

// An internal class - you should not be using this directly.
template <typename ValueT>
class JoystickElement {

public:

  // Type alias for callbacks
  using EventHandler = std::function<void(uint32_t, ValueT)>;

private:

  ValueT value_;
  uint32_t timestamp_;
  std::string name_;
  std::vector<EventHandler> callbacks_;
  std::mutex val_lock_;
  std::condition_variable cv_;

  void wait_for_next() {
    std::unique_lock<std::mutex> lock(val_lock_);
    cv_.wait(lock);
  }

public:

  JoystickElement() = delete;
  JoystickElement& operator=(const JoystickElement<ValueT>&) = delete;

  JoystickElement(const JoystickElement<ValueT>& o)
      : value_(o.value_), timestamp_(o.timestamp_), name_(o.name_),
        callbacks_(o.callbacks_) {}

  JoystickElement(JoystickElement<ValueT>&& o)
    : value_(o.value_), timestamp_(o.timestamp_), name_(std::move(o.name_)),
    callbacks_(std::move(o.callbacks_)) {}

  JoystickElement(const std::string& name="") : name_(name) {
    ValueT val{};
    update(0, val);
  }

  void update(uint32_t ts, ValueT value) {
    std::unique_lock<std::mutex> lock(val_lock_);

    value_ = value;
    timestamp_ = ts;

    for (auto& callback : callbacks_) {
      callback(ts, value);
    }

    cv_.notify_all();
  }

  void set_name(const std::string& name) {
    name_ = name;
  }

  const std::string name() const {
    return name_;
  }

  ValueT get() const {
    return value_;
  }

  uint32_t timestamp() const {
    return timestamp_;
  }

  ValueT get_next() {
    wait_for_next();
    return value_;
  }

  ValueT get_next(uint32_t& timestamp) {
    wait_for_next();
    timestamp = timestamp_;
    return value_;
  }

  void add_event_handler(EventHandler callback) {
    std::lock_guard<std::mutex> lock(val_lock_);
    callbacks_.push_back(callback);
  }

};

// Type alias for axis event callbacks
using AxisEventHandler = std::function<void(uint32_t, float)>;

// Type alias for hat event callbacks
using HatEventHandler = std::function<void(uint32_t, HatValue)>;

// Type alias for button event callbacks
using ButtonEventHandler = std::function<void(uint32_t, bool)>;

/**
 * 
 */
class Joystick {

private:

  struct ctor_key {};

  friend class JoystickDispatcher;
  friend class JoystickMapper;

  uint32_t num_axes_;
  uint32_t num_hats_;
  uint32_t num_buttons_;
  size_t index_;
  std::string name_;
  std::string guid_;
  SDL_Joystick* joystick_;
  SDL_GameController* game_controller_;

  std::vector<JoystickElement<float>> axis_events_;
  std::vector<JoystickElement<HatValue>> hat_events_;
  std::vector<JoystickElement<bool>> button_events_;

  std::map<std::string, size_t> axis_aliases_;
  std::map<std::string, size_t> button_aliases_;

  static void set_at(size_t index, SDL_Joystick* joystick, SDL_GameController* game_controller);

  void add_axis_alias(const char* alias, size_t axis);
  void add_button_alias(const char* alias, size_t button);
  void on_axis_event(uint32_t ts, size_t axis, float value);
  void on_hat_event(uint32_t ts, size_t hat, HatValue value);
  void on_button_event(uint32_t ts, size_t axis, bool value);

public:

//------------------------------------------------------------------------------

  // Do not use directly -- hence the `ctor_key`
  Joystick(size_t index, SDL_Joystick* joystick,
           SDL_GameController* game_controller, ctor_key);
  ~Joystick();

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
   * If the index is greater than the number of joysticks, a null shared pointer is returned.
   * If the joystick at the given index is not a game controller, a null shared pointer is returned.
   * 
   * @param index the index of the joystick to be retrieved. This corresponds
   *              to the index of the joystick/gamecontroller in the SDL2 API
   *              (e.g., SDL_JoystickOpen, SDL_GameControllerOpen)
   */
  static std::shared_ptr<Joystick> at_index(size_t index) noexcept;

  /**
   * Retrieves a vector of all available joysticks at the given point in time
   */
  static std::vector<std::shared_ptr<Joystick>> available_joysticks();

//------------------------------------------------------------------------------

  /**
   * The human readable string of the button at the given index.
   * If the provided index is out of bounds, "Invalid" is returned.
   */
  std::string get_button_name(size_t button) const noexcept;
  
  /**
   * The human readable string of the axis at the given index.
   * If the provided index is out of bounds, "Invalid" is returned.
   */
  std::string get_axis_name(size_t axis) const noexcept;

  /**
   * The human readable name of the joystick.
   * This is also the string returned by SDL_GameControllerName
   */
  std::string name() const noexcept;

  /**
   * The GUID returned by SDL_JoystickGetGUID in string form
   */
  std::string guid() const noexcept;

  /**
   * The index of this joystick in the SDL api at the time the
   * event handler API was initialized.
   */
  size_t index() const noexcept;

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
   * Adds a callback for the given hat. The provided callback is invoked
   * whenever a hat value change has been detected.
   */
  void add_hat_event_handler(size_t hat, HatEventHandler handler);

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
   * Retrieves the current value of the provided hat.
   * 
   * If the index is out of range (i.e., greater than/equal to the number of hats),
   * an exception is thrown.
   */
  HatValue get_current_hat_state(size_t hat);

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
   * Waits for the next detected movement of the hat and returns
   * the detected value.
   * 
   * If the index is out of range (i.e., greater than/equal to the number of hats),
   * an exception is thrown.
   */
  HatValue get_next_hat_state(size_t hat);

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
