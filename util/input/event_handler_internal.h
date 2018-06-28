#pragma once

#include "event_handler.h"

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>

namespace hebi {
namespace util {

using high_res_clock = std::chrono::high_resolution_clock;
using steady_clock = std::chrono::steady_clock;

// If high_resolution_clock is steady, use it. Otherwise, fallback to steady_clock.
using clock_type = typename std::conditional<high_res_clock::is_steady, high_res_clock, steady_clock>::type;

class SDLEventHandler {

private:

  std::map<SDL_EventType, std::vector<SDLEventCallback>> event_handlers_;
  std::mutex lock_;

  clock_type::time_point last_event_loop_time_;

  double event_loop_frequency_{0.0};
  uint64_t event_loop_period_us_{0};
  bool started_{false};
  bool keep_running_{true};
  std::condition_variable start_condition_;

  void dispatch_event(const SDL_Event& event);
  void run(std::condition_variable& cv);

public:

  SDLEventHandler();

  void set_loop_frequency(double frequency);
  void register_event(SDL_EventType event, SDLEventCallback callback);
  void start();
  void stop();

};

//------------------------------------------------------------------------------

void fail_on_not_init(const char* func);

}
}
