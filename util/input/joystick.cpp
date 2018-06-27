#include "joystick.h"

#include <cassert>
#include <condition_variable>
#include <mutex>

namespace hebi {
namespace util {
  
struct SDL2_InputSubsystemInitializer {
  SDL2_InputSubsystemInitializer() {
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
    SDL_JoystickEventState(SDL_ENABLE);
  }

  ~SDL2_InputSubsystemInitializer() {
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
  }
};

void load_joystick_system() {
  static SDL2_InputSubsystemInitializer loader;
}

static std::mutex sJoystickLock;
static std::vector<std::shared_ptr<Joystick>> sJoysticks;

//------------------------------------------------------------------------------

Joystick::Joystick(size_t index, SDL_Joystick* joystick,
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

Joystick::~Joystick() {
  if (joystick_) {
    SDL_JoystickClose(joystick_);
  }
  if (game_controller_) {
    SDL_GameControllerClose(game_controller_);
  }
}

// in joystick_mapper.cpp
void map_joystick(std::shared_ptr<Joystick> const& joy);

void Joystick::set_at(size_t index, SDL_Joystick* joystick, SDL_GameController* game_controller) {
  std::lock_guard<std::mutex> lock(sJoystickLock);
  if (sJoysticks.size() < (index + 1)) {
    size_t insert_count = (index + 1) - sJoysticks.size();
    for (size_t i = 0; i < insert_count; i++) {
      sJoysticks.push_back(std::shared_ptr<Joystick>(nullptr));
    }
  }
  auto joy = std::make_shared<Joystick>(index, joystick, game_controller, ctor_key{});
  map_joystick(joy);
  sJoysticks[index] = joy;
}

void Joystick::add_axis_alias(const char* alias, size_t axis) {
  axis_aliases_[std::string(alias)] = axis;
}

void Joystick::add_button_alias(const char* alias, size_t button) {
  button_aliases_[std::string(alias)] = button;
}

//------------------------------------------------------------------------------

size_t Joystick::joystick_count() {
  return static_cast<size_t>(SDL_NumJoysticks());
}

std::shared_ptr<Joystick> Joystick::at_index(size_t index) noexcept {
  if (index >= Joystick::joystick_count()) {
    return std::shared_ptr<Joystick>(nullptr);
  }
  std::lock_guard<std::mutex> lock(sJoystickLock);
  return sJoysticks[index];
}

std::vector<std::shared_ptr<Joystick>> Joystick::available_joysticks() {
  std::vector<std::shared_ptr<Joystick>> ret;
  std::lock_guard<std::mutex> lock(sJoystickLock);

  ret.reserve(sJoysticks.size());
  for (const auto& joystick : sJoysticks) {
    ret.push_back(joystick);
  }
  return ret;
}

//------------------------------------------------------------------------------

std::string Joystick::get_button_name(size_t button) const noexcept {
  if (button >= num_buttons_) {
    return "Invalid";
  }
  return button_events_[button].name();
}

std::string Joystick::get_axis_name(size_t axis) const noexcept {
  if (axis >= num_axes_) {
    return "Invalid";
  }
  return axis_events_[axis].name();
}

std::string Joystick::name() const noexcept {
  return name_;
}

std::string Joystick::guid() const noexcept {
  return guid_;
}

size_t Joystick::index() const noexcept {
  return index_;
}

//------------------------------------------------------------------------------

void Joystick::on_axis_event(uint32_t ts, size_t axis, float value) {
  assert(axis < num_axes_);
  axis_events_[axis].update(ts, value);
}

void Joystick::on_hat_event(uint32_t ts, size_t hat, HatValue value) {
  assert(hat < num_hats_);
  hat_events_[hat].update(ts, value);
}

void Joystick::on_button_event(uint32_t ts, size_t button, bool value) {
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

void Joystick::add_axis_event_handler(size_t axis, AxisEventHandler handler) {
  _throw_on_out_of_range(axis, num_axes_);
  axis_events_[axis].add_event_handler(std::move(handler));
}

void Joystick::add_axis_event_handler(const std::string& axis, AxisEventHandler handler) {
  auto itr = axis_aliases_.find(axis);
  if (axis_aliases_.end() == itr) {
    throw std::logic_error(axis + " is not a valid axis name");
  }
  axis_events_[itr->second].add_event_handler(std::move(handler));
}

void Joystick::add_hat_event_handler(size_t hat, HatEventHandler handler) {
  _throw_on_out_of_range(hat, num_hats_);
  hat_events_[hat].add_event_handler(std::move(handler));
}

void Joystick::add_button_event_handler(size_t button, ButtonEventHandler handler) {
  _throw_on_out_of_range(button, num_buttons_);
  button_events_[button].add_event_handler(std::move(handler));
}

void Joystick::add_button_event_handler(const std::string& button, ButtonEventHandler handler) {
  auto itr = button_aliases_.find(button);
  if (button_aliases_.end() == itr) {
    throw std::logic_error(button + " is not a valid button name");
  }
  button_events_[itr->second].add_event_handler(std::move(handler));
}

//------------------------------------------------------------------------------

float Joystick::get_current_axis_state(size_t axis) {
  _throw_on_out_of_range(axis, num_axes_);
  return axis_events_[axis].get();
}

float Joystick::get_current_axis_state(const std::string& axis) {
  return get_current_axis_state(axis_aliases_.at(axis));
}

HatValue Joystick::get_current_hat_state(size_t hat) {
  _throw_on_out_of_range(hat, num_hats_);
  return hat_events_[hat].get();
}

bool Joystick::get_current_button_state(size_t button) {
  _throw_on_out_of_range(button, num_buttons_);
  return button_events_[button].get();
}

bool Joystick::get_current_button_state(const std::string& button) {
  return get_current_button_state(button_aliases_.at(button));
}

//------------------------------------------------------------------------------

float Joystick::get_next_axis_state(size_t axis) {
  _throw_on_out_of_range(axis, num_axes_);
  return axis_events_[axis].get_next();
}

HatValue Joystick::get_next_hat_state(size_t hat) {
  _throw_on_out_of_range(hat, num_hats_);
  return hat_events_[hat].get_next();
}

bool Joystick::get_next_button_state(size_t button) {
  _throw_on_out_of_range(button, num_buttons_);
  return button_events_[button].get_next();
}

}
}
