#include "event_handler_internal.h"
#include "joystick_internal.h"

#include <stdexcept>

namespace hebi {
namespace util {

using JoystickImplPtr = std::unique_ptr<JoystickImpl, std::function<void(JoystickImpl*)>>;

struct LifecycleState {

  std::vector<JoystickImplPtr> joysticks_impl;
  std::vector<std::shared_ptr<Joystick>> joysticks;
  SDLEventHandler event_handler;

  LifecycleState() {}

  void init() {
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
    SDL_JoystickEventState(SDL_ENABLE);
    SDL_GameControllerEventState(SDL_ENABLE);

    register_event(SDL_CONTROLLERDEVICEADDED, JoystickDispatcher::controller_added);
    register_event(SDL_JOYHATMOTION, JoystickDispatcher::joystick_hat_event);
    register_event(SDL_JOYBUTTONDOWN, JoystickDispatcher::controller_button_event);
    register_event(SDL_JOYBUTTONUP, JoystickDispatcher::controller_button_event);
    register_event(SDL_JOYAXISMOTION, JoystickDispatcher::controller_axis_motion);

    event_handler.start();
  }

  ~LifecycleState() {
    event_handler.stop();   // Make sure to stop (and wait) for the
                            // event handler to finish running before deleting
                            // the joysticks. This way, we can be sure that no
                            // joystick buttons/axes event handlers will be invoked
                            // after this background thread has stopped.

    joysticks_impl.clear(); // This needs to be cleared before `joysticks`
    joysticks.clear();
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
  }

};

static std::recursive_mutex sSingletonLock;
static LifecycleState* sSingleton;
static bool sSingletonDestroyed{false}; // Used to prevent reinit of system

using recursive_lock_guard = std::lock_guard<std::recursive_mutex>;

//------------------------------------------------------------------------------
// Internal API

void fail_on_not_init(const char* func) {
  if (sSingleton == nullptr || sSingletonDestroyed) {
    sSingletonLock.unlock();
    std::string message = "Attempted to call function ";
    message += func;
    message += " when API was not initialized";
    fprintf(stderr, "%s\n", message.c_str());
    throw std::runtime_error(message);
  }
}

//------------------------------------------------------------------------------
// Joystick API

std::shared_ptr<Joystick> Joystick::at_index(size_t index) {
  recursive_lock_guard lock(sSingletonLock);
  fail_on_not_init(__func__);

  if (index >= Joystick::joystick_count()) {
    return std::shared_ptr<Joystick>(nullptr);
  }

  return sSingleton->joysticks[index];
}

std::vector<std::shared_ptr<Joystick>> Joystick::available_joysticks() {
  recursive_lock_guard lock(sSingletonLock);
  fail_on_not_init(__func__);
  std::vector<std::shared_ptr<Joystick>> ret;

  ret.reserve(sSingleton->joysticks.size());
  for (auto& joystick : sSingleton->joysticks) {
    ret.push_back(joystick);
  }
  return ret;
}

// Bound destructor for unique_ptr's below which are set to null
static void no_op_destructor(JoystickImpl*) {}

void JoystickImpl::set_at(size_t index, SDL_Joystick* joystick, SDL_GameController* game_controller) {
  recursive_lock_guard lock(sSingletonLock);
  fail_on_not_init(__func__);

  auto& joysticks = sSingleton->joysticks;
  auto& joysticks_impl = sSingleton->joysticks_impl;

  if (joysticks.size() < (index + 1)) {
    size_t insert_count = (index + 1) - joysticks.size();
    for (size_t i = 0; i < insert_count; i++) {
      joysticks.push_back(std::shared_ptr<Joystick>(nullptr));
      joysticks_impl.push_back(JoystickImplPtr(nullptr, no_op_destructor));
    }
  }

  auto destructor = [index] (JoystickImpl* impl) {
    Joystick::at_index(index)->dispose();
    delete impl;
  };
  auto joy_impl = JoystickImplPtr(new JoystickImpl(index, joystick, game_controller, JoystickImpl::ctor_key{}), destructor);
  auto joy = std::make_shared<Joystick>(
    joy_impl.get(),
    Joystick::ctor_key{});

  map_joystick(joy);
  joysticks_impl[index] = std::move(joy_impl);
  joysticks[index] = joy;
}

//------------------------------------------------------------------------------
// Public API

void register_event(SDL_EventType event, SDLEventCallback callback) {
  recursive_lock_guard lock(sSingletonLock);
  fail_on_not_init(__func__);
  sSingleton->event_handler.register_event(event, std::move(callback));
}

void initialize_event_handler() {
  recursive_lock_guard lock(sSingletonLock);
  if (sSingleton != nullptr || sSingletonDestroyed) {
    // Already initialized - bail out
    return;
  }

  sSingleton = new LifecycleState;
  sSingleton->init();
}

void quit_event_handler() {
  recursive_lock_guard lock(sSingletonLock);
  if (sSingleton == nullptr) {
    return;
  }

  delete sSingleton;
  sSingleton = nullptr;
  sSingletonDestroyed = true;
}

}
}
