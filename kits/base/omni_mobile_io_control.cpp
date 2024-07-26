
#include "util/mobile_io.hpp"
#include "util/omni_base.hpp"

using namespace hebi;
using namespace experimental;
using namespace mobile;


int main(int argc, char* argv[]) {

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////
  
  std::cout << "Creating MobileIO" << std::endl;

  std::string family = "Rosie";

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create(family, "mobileIO");
  while(!mobile) {
    std::cout << "Couldn't find mobileIO, trying again..." << std::endl;
    mobile = MobileIO::create(family, "mobileIO");
  }

  std::string instructions("A1/A2 - Move Base\n"
                           "A7 - Turn Base\n"
                           "B8 - Quit\n");

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Display instructions on screen
  mobile -> sendText(instructions); 

  //////////////////////////
  //// OmniBase Setup //////
  //////////////////////////

  // Set module names for mobile base
  OmniBase::Params p;
  p.families_ = {family};
  p.names_ = {"W1", "W2", "W3"};

  std::cout << "Creating Omni Base" << std::endl;

  auto base = OmniBase::create(p);

  if (!base) {
    std::cout << "Failed to create base, exiting!" << std::endl;
    exit(EXIT_FAILURE);
  }

  auto last_state = mobile->getState();

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(base->update())
  {
    auto state = mobile->getState();
    MobileIODiff diff(last_state, state);

    /////////////////
    // Input Handling
    /////////////////

    auto dy = -1 * state.getAxis(1);
    auto dx = state.getAxis(2);
    auto dtheta = -1 * state.getAxis(7);

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

    // create Vel Goal for 0.5 second, 0.25 second ramp down
    auto goal = CartesianGoal::createFromVelocity(Vel{dx, dy, dtheta}, 0.1, 0.5, 0.25);

    // send goal to base
    base->setGoal(goal);

    // Update to the new last_state for mobile device
    last_state = state;

    // Send latest commands to the base
    base->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
};
