
#include "util/omni_base.hpp"

using namespace hebi;
using namespace experimental;
using namespace mobile;


int main(int argc, char* argv[]) {

  //////////////////////////
  //// OmniBase Setup //////
  //////////////////////////

  // Set module names for mobile base
  OmniBase::Params p;
  p.families_ = {"Rosie"};
  p.names_ = {"W1", "W2", "W3"};

  std::cout << "Creating Omni Base" << std::endl;

  auto base = OmniBase::create(p);

  if (!base) {
    std::cout << "Failed to create base, exiting!" << std::endl;
    exit(EXIT_FAILURE);
  }

  auto currentPose = base->getOdometry();

  Waypoint wp1;
  Waypoint wp2;
  Waypoint wp3;
  Waypoint wp4;

  wp1.t = 1.0;
  wp1.pos = currentPose;

  wp2 = wp1;
  wp2.t += 1.0;
  wp2.pos.x += 0.5;

  wp3 = wp2;
  wp3.t += 1.0;
  wp3.pos.y += 0.5;

  wp4 = wp3;
  wp4.t += 1.0;
  wp4.pos.x -= 0.5;

  auto goal = CartesianGoal::createFromWaypoints({wp1, wp2, wp3, wp4});

  // send goal to base
  base->setGoal(goal);

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  std::cout << "Executing Goal" << std::endl;
  return -1;

  while (base->update())
  {
    // Send updated command to the base
    base->send();

    // if the trajectory has been completed, start another square
    if (base->goalComplete()) {
      base->setGoal(goal);
    }
  }

  return 0;
};
