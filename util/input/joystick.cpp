#include "joystick_internal.h"

#include <cstdio>
#include <limits>

namespace hebi {
namespace util {

std::unique_lock<std::mutex> Joystick::scoped_lock() {
  std::unique_lock<std::mutex> ret(lifecycle_lock_);
  return ret;
}

bool Joystick::is_disposed(const char* func) const {
  // NOTE:
  //  This function assumes that lifecycle_lock_ has already been acquired.
  bool ret = impl_ == nullptr;
  if (ret) {
    fprintf(stderr, "Attempted to call 'Joystick::%s' on already disposed Joystick\n", func);
  }
  return ret;
}

void Joystick::dispose() {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (impl_ == nullptr) {
    return;
  }
  impl_ = nullptr;
}

//------------------------------------------------------------------------------

size_t Joystick::joystick_count() {
  return static_cast<size_t>(SDL_NumJoysticks());
}

//------------------------------------------------------------------------------

std::string Joystick::get_button_name(size_t button) const {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return "";
  }
  return impl_->get_button_name(button);
}
  
std::string Joystick::get_axis_name(size_t axis) const {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return "";
  }
  return impl_->get_axis_name(axis);
}

std::string Joystick::name() const {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return "";
  }
  return impl_->name();
}

std::string Joystick::guid() const {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return "";
  }
  return impl_->guid();
}

size_t Joystick::index() const {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return std::numeric_limits<size_t>::max();
  }
  return impl_->index();
}

//------------------------------------------------------------------------------

void Joystick::add_axis_event_handler(size_t axis, AxisEventHandler handler) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return;
  }
  impl_->add_axis_event_handler(axis, std::move(handler));
}

void Joystick::add_axis_event_handler(const std::string& axis, AxisEventHandler handler) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return;
  }
  impl_->add_axis_event_handler(axis, std::move(handler));
}

void Joystick::add_button_event_handler(size_t button, ButtonEventHandler handler) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return;
  }
  impl_->add_button_event_handler(button, std::move(handler));
}

void Joystick::add_button_event_handler(const std::string& button, ButtonEventHandler handler) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return;
  }
  impl_->add_button_event_handler(button, std::move(handler));
}

//------------------------------------------------------------------------------

float Joystick::get_current_axis_state(size_t axis) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return impl_->get_current_axis_state(axis);
}

float Joystick::get_current_axis_state(const std::string& axis) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return impl_->get_current_axis_state(axis);
}

bool Joystick::get_current_button_state(size_t button) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return false;
  }
  return impl_->get_current_button_state(button);
}

bool Joystick::get_current_button_state(const std::string& button) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return false;
  }
  return impl_->get_current_button_state(button);
}

//------------------------------------------------------------------------------

float Joystick::get_next_axis_state(size_t axis) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return false;
  }
  return impl_->get_next_axis_state(axis);
}

bool Joystick::get_next_button_state(size_t button) {
  std::lock_guard<std::mutex> lock(lifecycle_lock_);
  if (is_disposed(__func__)) {
    return false;
  }
  return impl_->get_next_button_state(button);
}

}
}
