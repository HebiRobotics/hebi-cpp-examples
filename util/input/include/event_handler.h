#pragma once

#include <SDL.h>
#include <functional>

namespace hebi {
namespace util {

using SDLEventCallback = std::function<void(const SDL_Event&)>;

/**
 * Register an event handler for the given SDL event type
 * 
 * This is a lower level mechanism used to register an event handler.
 * This can be used to extend functionality using the SDL2 API.
 * If you want to register a callback for a Joystick, you should
 * use the Joystick API instead.
 * 
 * @param event The SDL2 event type
 * @param callback The function invoked when an event of the given type is received
 */
void register_event(SDL_EventType event, SDLEventCallback callback);

/**
 * Initializes the event handler subsystem.
 * 
 * This function must be called before any other function in the
 * event handler API.
 */
void initialize_event_handler();

}
}
