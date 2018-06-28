#pragma once

#include "joystick.h"

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

//------------------------------------------------------------------------------

/**
 * 
 */
class JoystickImpl {

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

  uint8_t sdl_gamecontroller_to_index_[15];

  std::vector<JoystickElement<float>> axis_events_;
  std::vector<JoystickElement<HatValue>> hat_events_;
  std::vector<JoystickElement<bool>> button_events_;

  std::map<std::string, size_t> axis_aliases_;
  std::map<std::string, size_t> button_aliases_;

public:

  // Do not use directly -- hence the `ctor_key`
  JoystickImpl(size_t index, SDL_Joystick* joystick,
               SDL_GameController* game_controller, ctor_key);
  ~JoystickImpl();

  // Explicitly deleted to ensure only one constructor signature is allowed
  JoystickImpl() = delete;
  JoystickImpl(const JoystickImpl&) = delete;
  JoystickImpl(JoystickImpl&&) = delete;

//------------------------------------------------------------------------------
// Internal functions

  static void set_at(size_t index, SDL_Joystick* joystick, SDL_GameController* game_controller);

  void add_axis_alias(const char* alias, size_t axis);
  void add_button_alias(const char* alias, size_t button);
  void on_axis_event(uint32_t ts, size_t axis, float value);
  void on_hat_event(uint32_t ts, size_t hat, HatValue value);
  void on_button_event(uint32_t ts, size_t axis, bool value);

//------------------------------------------------------------------------------
// public Joystick delegate functions

  bool has_mapping() const noexcept;
  std::string get_button_name(size_t button) const noexcept;
  std::string get_axis_name(size_t axis) const noexcept;
  std::string name() const noexcept;
  std::string guid() const noexcept;
  size_t index() const noexcept;

  void add_axis_event_handler(size_t axis, AxisEventHandler handler);
  void add_axis_event_handler(const std::string& axis, AxisEventHandler handler);
  void add_hat_event_handler(size_t hat, HatEventHandler handler);
  void add_button_event_handler(size_t button, ButtonEventHandler handler);
  void add_button_event_handler(const std::string& button, ButtonEventHandler handler);

  float get_current_axis_state(size_t axis);
  float get_current_axis_state(const std::string& axis);
  HatValue get_current_hat_state(size_t hat);
  bool get_current_button_state(size_t button);
  bool get_current_button_state(const std::string& button);

  float get_next_axis_state(size_t axis);
  HatValue get_next_hat_state(size_t hat);
  bool get_next_button_state(size_t button);

};

//------------------------------------------------------------------------------

class JoystickDispatcher {

public:

  static void controller_added(const SDL_Event& event);
  static void joystick_hat_event(const SDL_Event& event);
  static void controller_button_event(const SDL_Event& event);
  static void controller_axis_motion(const SDL_Event& event);

};

void map_joystick(std::shared_ptr<Joystick> const& joy);

}
}
