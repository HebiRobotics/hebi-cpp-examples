#include "event_handler_internal.h"
#include "joystick.h"

#include <cassert>
#include <cmath>
#include <thread>
#include <type_traits>

namespace hebi {
namespace util {

template<typename T>
uint64_t count_micros(T&& d) {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(d).count());
}

static void sleep_micros(uint64_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

//------------------------------------------------------------------------------
// Internal event handler class

void SDLEventHandler::dispatch_event(const SDL_Event& event) {
  SDL_EventType type = static_cast<SDL_EventType>(event.type);
  auto iter = event_handlers_.find(type);
  if (iter == event_handlers_.end()) {
    return;
  }

  for (auto& callback : iter->second) {
    callback(event);
  }
}

void SDLEventHandler::run(std::condition_variable& cv) {
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

  lock_.lock();
  while(keep_running_) {
    lock_.unlock();
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

    lock_.lock();
  }

  lock_.unlock();
  cv.notify_all();
}

SDLEventHandler::SDLEventHandler() {
  set_loop_frequency(200.0);
}

void SDLEventHandler::set_loop_frequency(double frequency) {
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

void SDLEventHandler::register_event(SDL_EventType event, SDLEventCallback callback) {
  std::lock_guard<std::mutex> lock(lock_);
  auto& handlers = event_handlers_[event];

  handlers.push_back(callback);
}

void SDLEventHandler::start() {
  std::unique_lock<std::mutex> lock(lock_);
  if (started_) {
    return;
  }

  std::thread proc_thread(&SDLEventHandler::run, this, std::ref(start_condition_));
  proc_thread.detach();
  start_condition_.wait(lock);
}

void SDLEventHandler::stop() {
  std::unique_lock<std::mutex> lock(lock_);
  keep_running_ = false;
  start_condition_.wait(lock);
}

}
}
