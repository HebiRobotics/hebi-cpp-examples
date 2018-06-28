#include "joystick_internal.h"

#include <iostream>
#include <stdexcept>

namespace hebi {
namespace util {

/**
 * Internal class used to map names to axes and buttons
 */
class JoystickMapper {

public:

  static void map_joystick(std::shared_ptr<Joystick> const& joy) {
    auto joy_impl = joy->impl_;
    auto game_controller = joy->impl_->game_controller_;

    // See if the game controller has a mapping
    char* str_mapping = SDL_GameControllerMapping(game_controller);
    if (str_mapping == nullptr) {
      if (getenv("HEBI_DEBUG")) {
        printf("No mapping exists for controller %s at index %zu (GUID: %s)\n",
               joy->name().c_str(), joy->index(), joy->guid().c_str());
      }

      return;
    }

    SDL_free(str_mapping);

    int btn_a_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_A).value.button;
    int btn_b_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_B).value.button;
    int btn_x_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_X).value.button;
    int btn_y_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_Y).value.button;
    int btn_back_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_BACK).value.button;
    int btn_guide_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_GUIDE).value.button;
    int btn_start_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_START).value.button;
    int btn_leftstick_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_LEFTSTICK).value.button;
    int btn_rightstick_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_RIGHTSTICK).value.button;
    int btn_leftshoulder_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER).value.button;
    int btn_rightshoulder_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER).value.button;
    int btn_dpad_up_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_DPAD_UP).value.button;
    int btn_dpad_down_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN).value.button;
    int btn_dpad_left_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT).value.button;
    int btn_dpad_right_index = SDL_GameControllerGetBindForButton(game_controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT).value.button;

    int axs_left_x_index = SDL_GameControllerGetBindForAxis(game_controller, SDL_CONTROLLER_AXIS_LEFTX).value.axis;
    int axs_left_y_index = SDL_GameControllerGetBindForAxis(game_controller, SDL_CONTROLLER_AXIS_LEFTY).value.axis;
    int axs_right_x_index = SDL_GameControllerGetBindForAxis(game_controller, SDL_CONTROLLER_AXIS_RIGHTX).value.axis;
    int axs_right_y_index = SDL_GameControllerGetBindForAxis(game_controller, SDL_CONTROLLER_AXIS_RIGHTY).value.axis;
    int axs_left_trigger_index = SDL_GameControllerGetBindForAxis(game_controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT).value.axis;
    int axs_right_trigger_index = SDL_GameControllerGetBindForAxis(game_controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT).value.axis;

    joy_impl->add_axis_alias("LEFT_STICK_X", axs_left_x_index);
    joy_impl->add_axis_alias("LEFT_STICK_Y", axs_left_y_index);
    joy_impl->add_axis_alias("RIGHT_STICK_X", axs_right_x_index);
    joy_impl->add_axis_alias("RIGHT_STICK_Y", axs_right_y_index);
    joy_impl->add_axis_alias("LEFT_TRIGGER", axs_left_trigger_index);
    joy_impl->add_axis_alias("RIGHT_TRIGGER", axs_right_trigger_index);
    // other common names for axes
    joy_impl->add_axis_alias("L2", axs_left_trigger_index);  // Xbox/PS3/PS4
    joy_impl->add_axis_alias("R2", axs_right_trigger_index); // Xbox/PS3/PS4

    joy_impl->add_button_alias("A", btn_a_index);
    joy_impl->add_button_alias("B", btn_b_index);
    joy_impl->add_button_alias("X", btn_x_index);
    joy_impl->add_button_alias("Y", btn_y_index);
    joy_impl->add_button_alias("BACK", btn_back_index);
    joy_impl->add_button_alias("GUIDE", btn_guide_index);
    joy_impl->add_button_alias("START", btn_start_index);
    joy_impl->add_button_alias("LEFT_STICK", btn_leftstick_index);
    joy_impl->add_button_alias("RIGHT_STICK", btn_rightstick_index);
    joy_impl->add_button_alias("LEFT_SHOULDER", btn_leftshoulder_index);
    joy_impl->add_button_alias("RIGHT_SHOULDER", btn_rightshoulder_index);
    joy_impl->add_button_alias("DPAD_UP", btn_dpad_up_index);
    joy_impl->add_button_alias("DPAD_DOWN", btn_dpad_down_index);
    joy_impl->add_button_alias("DPAD_LEFT", btn_dpad_left_index);
    joy_impl->add_button_alias("DPAD_RIGHT", btn_dpad_right_index);
    // other common names for buttons
    joy_impl->add_button_alias("X_PLAYSTATION", btn_a_index);        // PS3/PS4
    joy_impl->add_button_alias("CIRCLE_PLAYSTATION", btn_b_index);   // PS3/PS4
    joy_impl->add_button_alias("SQUARE_PLAYSTATION", btn_x_index);   // PS3/PS4
    joy_impl->add_button_alias("TRIANGLE_PLAYSTATION", btn_y_index); // PS3/PS4
    joy_impl->add_button_alias("SELECT", btn_back_index);            // PS3
    joy_impl->add_button_alias("SHARE", btn_back_index);             // PS4
    joy_impl->add_button_alias("TOUCHPAD", btn_guide_index);         // PS4
    joy_impl->add_button_alias("OPTIONS", btn_start_index);          // PS4
    joy_impl->add_button_alias("L3", btn_leftstick_index);           // Xbox/PS3/PS4
    joy_impl->add_button_alias("R3", btn_rightstick_index);          // Xbox/PS3/PS4
    joy_impl->add_button_alias("L1", btn_leftshoulder_index);        // Xbox/PS3/PS4
    joy_impl->add_button_alias("R1", btn_rightshoulder_index);       // Xbox/PS3/PS4

  }

};

void map_joystick(std::shared_ptr<Joystick> const& joy) {
  JoystickMapper::map_joystick(joy);
}

}
}
