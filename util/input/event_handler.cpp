#include "event_handler.h"
#include "joystick.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <thread>
#include <type_traits>
#include <vector>

namespace hebi {
namespace util {

using high_res_clock = std::chrono::high_resolution_clock;
using steady_clock = std::chrono::steady_clock;

// If high_resolution_clock is steady, use it. Otherwise, fallback to steady_clock.
using clock_type = typename std::conditional<high_res_clock::is_steady, high_res_clock, steady_clock>::type;

template<typename T>
uint64_t count_micros(T&& d) {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(d).count());
}

static void sleep_micros(uint64_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

//------------------------------------------------------------------------------
// Internal event handler class

class SDLEventHandler {

private:

  std::map<SDL_EventType, std::vector<SDLEventCallback>> event_handlers_;
  std::mutex lock_;

  clock_type::time_point last_event_loop_time_;

  double event_loop_frequency_{0.0};
  uint64_t event_loop_period_us_{0};
  bool started_{false};
  bool keep_running_{true};

  void dispatch_event(const SDL_Event& event) {
    SDL_EventType type = static_cast<SDL_EventType>(event.type);
    auto iter = event_handlers_.find(type);
    if (iter == event_handlers_.end()) {
      return;
    }

    for (auto& callback : iter->second) {
      callback(event);
    }
  }

  void run(std::condition_variable& cv) {
    // modify state to signal the event handler has begun running
    {
      std::unique_lock<std::mutex> lk(lock_);
      started_ = true;
    }
    cv.notify_all();

    int numevents = 10;
    SDL_eventaction op = SDL_GETEVENT;
    Uint32 first = SDL_FIRSTEVENT;
    Uint32 last = SDL_LASTEVENT;

    SDL_Event events[10];

    while(keep_running_) {
      auto last_time = last_event_loop_time_;
      auto now_time = clock_type::now();
      uint64_t dt = count_micros(now_time - last_time);
      // Limit the rate at which events are pumped
      if (dt < event_loop_period_us_) {
        sleep_micros(event_loop_period_us_-dt);
        last_event_loop_time_ = clock_type::now();
      } else {
        last_event_loop_time_ = now_time;
      }

      SDL_PumpEvents();

      int readevents = 0;
      while(true) {
        readevents = SDL_PeepEvents(events, numevents, op, first, last);
        if (readevents < 1) {
          break;
        }
        for (size_t i = 0; i < static_cast<size_t>(readevents); i++) {
          dispatch_event(events[i]);
        }
      }

    }

  }

public:

  SDLEventHandler() {
    set_loop_frequency(200.0);
  }

  void set_loop_frequency(double frequency) {
    if (frequency <= 0.0 || !std::isfinite(frequency)) {
      return;
    }
    if (frequency > 500.0) {
      frequency = 500.0;
    }

    event_loop_frequency_ = frequency;
    double period_microseconds = (1.0 / frequency) * 1000.0 * 1000.0;
    event_loop_period_us_ = static_cast<uint64_t>(std::ceil(period_microseconds));
  }

  void register_event(SDL_EventType event, SDLEventCallback callback) {
    std::lock_guard<std::mutex> lock(lock_);
    auto& handlers = event_handlers_[event];

    handlers.push_back(callback);
  }

  void start() {
    std::unique_lock<std::mutex> lock(lock_);
    if (started_) {
      return;
    }

    std::condition_variable start_condition;
    std::thread proc_thread(&SDLEventHandler::run, this, std::ref(start_condition));
    proc_thread.detach();
    start_condition.wait(lock);
  }

};

static SDLEventHandler sSingleton; // NOLINT

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

class JoystickDispatcher {

public:

  static void joystick_added(const SDL_Event& event) {
    const auto& joystick_event = event.jdevice;
    auto which = joystick_event.which;

    if (!SDL_IsGameController(which)) {
      // This joystick is not supported by the game controller interface
      return;
    }
    auto gamecontroller = SDL_GameControllerOpen(which);
    assert(gamecontroller != nullptr);

    auto joystick = SDL_JoystickOpen(which);
    Joystick::set_at(which, joystick, gamecontroller);
  }

  static void joystick_axis_event(const SDL_Event& event) {
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

    joystick->on_axis_event(ts, axis, axis_value(axis_event.value));
  }

  static void joystick_hat_event(const SDL_Event& event) {
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

    joystick->on_hat_event(ts, hat, hat_value(hat_event.value));
  }

  static void joystick_button_event(const SDL_Event& event) {
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

    joystick->on_button_event(ts, button, button_value(button_event.state));
  }

};

//------------------------------------------------------------------------------
// Event handler initializer

// from joystick.cpp
void load_joystick_system();

struct EventHandlerInitializer {

  EventHandlerInitializer() {
    load_joystick_system();

    register_event(SDL_JOYAXISMOTION, JoystickDispatcher::joystick_axis_event);
    register_event(SDL_JOYHATMOTION, JoystickDispatcher::joystick_hat_event);
    register_event(SDL_JOYBUTTONDOWN, JoystickDispatcher::joystick_button_event);
    register_event(SDL_JOYBUTTONUP, JoystickDispatcher::joystick_button_event);
    register_event(SDL_JOYDEVICEADDED, JoystickDispatcher::joystick_added);

    sSingleton.start();
  }

};

//------------------------------------------------------------------------------
// Public API

void initialize_event_handler() {
  // a static variable within the scope of a function is guaranteed to be instantiated only once
  // in a thread safe manner. See: 
  //   https://en.cppreference.com/w/cpp/language/storage_duration#Static_local_variables
  // for more info.
  static EventHandlerInitializer initializer{};
}

void register_event(SDL_EventType event, SDLEventCallback callback) {
  sSingleton.register_event(event, std::move(callback));
}

}
}
