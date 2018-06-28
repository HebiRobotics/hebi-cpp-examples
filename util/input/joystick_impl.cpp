#include "joystick_internal.h"

#include <cassert>
#include <condition_variable>
#include <mutex>

namespace hebi {
namespace util {

//------------------------------------------------------------------------------

JoystickImpl::JoystickImpl(size_t index, SDL_Joystick* joystick,
    SDL_GameController* game_controller, ctor_key /*unused*/)
  : index_(index), joystick_(joystick), game_controller_(game_controller) {
  num_axes_ = static_cast<uint32_t>(SDL_JoystickNumAxes(joystick));
  num_hats_ = static_cast<uint32_t>(SDL_JoystickNumHats(joystick));
  num_buttons_ = static_cast<uint32_t>(SDL_JoystickNumButtons(joystick));

  axis_events_.reserve(num_axes_);
  hat_events_.reserve(num_hats_);
  button_events_.reserve(num_buttons_);

  for (size_t i = 0; i < num_axes_; i++) {
    axis_events_.emplace_back("");
  }
  for (size_t i = 0; i < num_hats_; i++) {
    hat_events_.emplace_back("");
  }
  for (size_t i = 0; i < num_buttons_; i++) {
    button_events_.emplace_back("");
  }

  name_ = SDL_GameControllerName(game_controller);

  {
    guid_.reserve(32);
    char buffer[33];
    SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(joystick),
                              buffer, 33);
    guid_.append(buffer);
  }

}

JoystickImpl::~JoystickImpl() {
  //SDL_JoystickClose(joystick_); // GameController API takes care of this already
  SDL_GameControllerClose(game_controller_);
}

//------------------------------------------------------------------------------

void JoystickImpl::add_axis_alias(const char* alias, size_t axis) {
  axis_aliases_[std::string(alias)] = axis;
}

void JoystickImpl::add_button_alias(const char* alias, size_t button) {
  button_aliases_[std::string(alias)] = button;
}

//------------------------------------------------------------------------------

bool JoystickImpl::has_mapping() const noexcept {
  return !(axis_aliases_.empty() && button_aliases_.empty());
}

std::string JoystickImpl::get_button_name(size_t button) const noexcept {
  if (button >= num_buttons_) {
    return "Invalid";
  }
  return button_events_[button].name();
}

std::string JoystickImpl::get_axis_name(size_t axis) const noexcept {
  if (axis >= num_axes_) {
    return "Invalid";
  }
  return axis_events_[axis].name();
}

std::string JoystickImpl::name() const noexcept {
  return name_;
}

std::string JoystickImpl::guid() const noexcept {
  return guid_;
}

size_t JoystickImpl::index() const noexcept {
  return index_;
}

//------------------------------------------------------------------------------

void JoystickImpl::on_axis_event(uint32_t ts, size_t axis, float value) {
  assert(axis < num_axes_);
  axis_events_[axis].update(ts, value);
}

void JoystickImpl::on_hat_event(uint32_t ts, size_t hat, HatValue value) {
  assert(hat < num_hats_);
  hat_events_[hat].update(ts, value);
}

void JoystickImpl::on_button_event(uint32_t ts, size_t button, bool value) {
  assert(button < num_buttons_);
  button_events_[button].update(ts, value);
}

//------------------------------------------------------------------------------

static inline void _throw_on_out_of_range(size_t index, size_t length) {
  if (index >= length) {
    std::string err_msg = std::to_string(index) +
      " is out of range (length = " + std::to_string(length) + ")";
    throw std::out_of_range(err_msg);
  }
}

void JoystickImpl::add_axis_event_handler(size_t axis, AxisEventHandler handler) {
  _throw_on_out_of_range(axis, num_axes_);
  axis_events_[axis].add_event_handler(std::move(handler));
}

void JoystickImpl::add_axis_event_handler(const std::string& axis, AxisEventHandler handler) {
  auto itr = axis_aliases_.find(axis);
  if (axis_aliases_.end() == itr) {
    throw std::logic_error(axis + " is not a valid axis name");
  }
  axis_events_[itr->second].add_event_handler(std::move(handler));
}

void JoystickImpl::add_hat_event_handler(size_t hat, HatEventHandler handler) {
  _throw_on_out_of_range(hat, num_hats_);
  hat_events_[hat].add_event_handler(std::move(handler));
}

void JoystickImpl::add_button_event_handler(size_t button, ButtonEventHandler handler) {
  _throw_on_out_of_range(button, num_buttons_);
  button_events_[button].add_event_handler(std::move(handler));
}

void JoystickImpl::add_button_event_handler(const std::string& button, ButtonEventHandler handler) {
  auto itr = button_aliases_.find(button);
  if (button_aliases_.end() == itr) {
    throw std::logic_error(button + " is not a valid button name");
  }
  button_events_[itr->second].add_event_handler(std::move(handler));
}

//------------------------------------------------------------------------------

float JoystickImpl::get_current_axis_state(size_t axis) {
  _throw_on_out_of_range(axis, num_axes_);
  return axis_events_[axis].get();
}

float JoystickImpl::get_current_axis_state(const std::string& axis) {
  return get_current_axis_state(axis_aliases_.at(axis));
}

HatValue JoystickImpl::get_current_hat_state(size_t hat) {
  _throw_on_out_of_range(hat, num_hats_);
  return hat_events_[hat].get();
}

bool JoystickImpl::get_current_button_state(size_t button) {
  _throw_on_out_of_range(button, num_buttons_);
  return button_events_[button].get();
}

bool JoystickImpl::get_current_button_state(const std::string& button) {
  return get_current_button_state(button_aliases_.at(button));
}

//------------------------------------------------------------------------------

float JoystickImpl::get_next_axis_state(size_t axis) {
  _throw_on_out_of_range(axis, num_axes_);
  return axis_events_[axis].get_next();
}

HatValue JoystickImpl::get_next_hat_state(size_t hat) {
  _throw_on_out_of_range(hat, num_hats_);
  return hat_events_[hat].get_next();
}

bool JoystickImpl::get_next_button_state(size_t button) {
  _throw_on_out_of_range(button, num_buttons_);
  return button_events_[button].get_next();
}

//------------------------------------------------------------------------------
// Built in event handlers

static inline float axis_value(int16_t value) {
  return static_cast<float>(static_cast<double>(value) * 0.0000305185);
}

static inline HatValue hat_value(uint8_t value) {
  HatValue ret; // NOLINT

  if (value & SDL_HAT_UP) {
    ret.y = 1;
  } else if (value & SDL_HAT_DOWN) {
    ret.y = -1;
  } else {
    ret.y = 0;
  }

  if (value & SDL_HAT_RIGHT) {
    ret.x = 1;
  } else if (value & SDL_HAT_LEFT) {
    ret.x = -1;
  } else {
    ret.x = 0;
  }

  return ret;
}

static inline bool button_value(uint8_t value) {
  return value == SDL_PRESSED;
}


void JoystickDispatcher::controller_added(const SDL_Event& event) {
  const auto& joystick_event = event.cdevice;
  auto which = joystick_event.which;

  if (!SDL_IsGameController(which)) {
    // This joystick is not supported by the game controller interface
    return;
  }
  auto gamecontroller = SDL_GameControllerOpen(which);
  assert(gamecontroller != nullptr);

  auto joystick = SDL_JoystickOpen(which);
  JoystickImpl::set_at(which, joystick, gamecontroller);
}

void JoystickDispatcher::joystick_hat_event(const SDL_Event& event) {
  const auto& hat_event = event.jhat;
  auto ts = hat_event.timestamp;
  auto which = hat_event.which;
  size_t hat = hat_event.hat;

  auto _joystick = Joystick::at_index(which);
  auto joystick = _joystick.get();
  assert(joystick != nullptr);
  if (joystick == nullptr) {
    return;
  }

  auto lock = joystick->scoped_lock();
  auto joystick_impl = joystick->impl_;
  if (joystick_impl == nullptr) {
    return;
  }
  joystick_impl->on_hat_event(ts, hat, hat_value(hat_event.value));
}

void JoystickDispatcher::controller_button_event(const SDL_Event& event) {
  const auto& button_event = event.jbutton;
  auto ts = button_event.timestamp;
  auto which = button_event.which;
  size_t button = button_event.button;

  auto _joystick = Joystick::at_index(which);
  auto joystick = _joystick.get();
  assert(joystick != nullptr);
  if (joystick == nullptr) {
    return;
  }

  auto lock = joystick->scoped_lock();
  auto joystick_impl = joystick->impl_;
  if (joystick_impl == nullptr) {
    return;
  }
  joystick_impl->on_button_event(ts, button, button_value(button_event.state));
}

void JoystickDispatcher::controller_axis_motion(const SDL_Event& event) {
  const auto& axis_event = event.jaxis;
  auto ts = axis_event.timestamp;
  auto which = axis_event.which;
  size_t axis = axis_event.axis;

  auto _joystick = Joystick::at_index(which);
  auto joystick = _joystick.get();
  assert(joystick != nullptr);
  if (joystick == nullptr) {
    return;
  }

  auto lock = joystick->scoped_lock();
  auto joystick_impl = joystick->impl_;
  if (joystick_impl == nullptr) {
    return;
  }
  joystick_impl->on_axis_event(ts, axis, axis_value(axis_event.value));
}

}
}
