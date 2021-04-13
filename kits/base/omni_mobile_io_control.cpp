
#include "util/mobile_io.hpp"
#include "util/omni_base.hpp"

using namespace hebi;
using namespace experimental;
using namespace mobile;


int main(int argc, char* argv[]) {

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("HEBI", "mobileIO");

  std::string instructions;
  instructions = ("B1 - Waypoint 1\nB2 - Waypoint 2\n"
                  "B3 - Waypoint 3\n"
                  "B6 - Grav comp mode\nB8 - Quit\n");
  // Clear any garbage on screen
  mobile -> clearText(); 

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup instructions
  auto last_state = mobile->getState();

  //////////////////////////
  //// OmniBase Setup //////
  //////////////////////////

  // Set module names for mobile base
  OmniBase::Params p;
  p.families_ = {"HEBI"};
  p.names_ = {"W1", "W2", "W3"};

  OmniBase base(p);

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(base.update())
  {

    auto state = mobile->getState();
    MobileIODiff diff(last_state, state);

    /////////////////
    // Button Presses
    /////////////////

    auto dy = state.getAxis(1);
    auto dx = state.getAxis(2);
    auto dtheta = state.getAxis(7);

    // Button B8 - End Demo
    if (diff.get(8) == MobileIODiff::ButtonState::ToOn)
    {
      // Clear MobileIO text
      mobile->clearText();
      return 1;
    }

    /////////////////
    // Update & send
    /////////////////

    // create Base Goal (theta = 1), for 1 second, 0.5 second ramp down
    auto goal = CartesianGoal::createFromVelocity(Vel{dx, dy, dtheta}, 0.5, 0.25);

    // send goal to base
    base.setGoal(goal);

    // Update to the new last_state for mobile device
    last_state = state;

    // Send latest commands to the base
    base.send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
};