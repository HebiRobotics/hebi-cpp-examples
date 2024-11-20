/**
 * Mobile IO Control
 * An example for setting up your arm for simple control from a mobile io devoce
 * to pre-programmed waypoints.
 */

// #include "lookup.hpp"
// #include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>

using namespace hebi;
using namespace experimental; 

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {"Arm"};
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};

  // Read HRDF file to seutp a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  params.hrdf_file_ = "kits/arm/hrdf/A-2085-06.hrdf";

  // Create the Arm Object
  auto arm = arm::Arm::create(params);
  while (!arm) {
    arm = arm::Arm::create(params);
  }

  // Load the gains file that is approriate to the arm
  arm -> loadGains("kits/arm/gains/A-2085-06.xml");

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<util::MobileIO> mobile = util::MobileIO::create(params.families_[0], "mobileIO");
  if (!mobile)
  {
    std::cout << "couldn't find mobile IO device!\n";
    return 1;
  }

  std::string instructions;
  instructions = ("B1 - Waypoint 1\nB2 - Waypoint 2\n"
                  "B3 - Waypoint 3\n"
                  "B6 - Grav comp mode\nB8 - Quit\n");
  // Clear any garbage on screen
  mobile->clearText(); 

  // Display instructions on screen
  mobile->appendText(instructions); 

  // Setup instructions
  auto last_state = mobile->update();

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Single Waypoint Vectors
  auto num_joints = arm->robotModel().getDoFCount();
  Eigen::VectorXd positions(num_joints);
  double single_time;
  single_time = 3;

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update())
  {
    auto updated_mobile = mobile->update(0);

    if (!updated_mobile)
      std::cout << "Failed to get feedback from mobile I/O; check connection!\n";
    else
    {
      /////////////////
      // Button Presses
      /////////////////

      // Buttton B1 - Home Position
      if (mobile->getButtonDiff(1) == util::MobileIO::ButtonState::ToOn) {
        positions << 0, 0, 0, 0, 0, 0;
        arm -> setGoal(arm::Goal::createFromPosition(single_time, positions));
      }

      // Button B2 - Waypoint 1
      if (mobile->getButtonDiff(2) == util::MobileIO::ButtonState::ToOn) {
        positions << M_PI/4, M_PI/3, 2*M_PI/3, M_PI/3, M_PI/4, 0;
        arm -> setGoal(arm::Goal::createFromPosition(single_time, positions));
      }

      // Button B3 - Waypoint 2
      if (mobile->getButtonDiff(3) == util::MobileIO::ButtonState::ToOn) {
        positions << -M_PI/4, M_PI/3, 2*M_PI/3, M_PI/3, 3*M_PI/4, 0;
        arm -> setGoal(arm::Goal::createFromPosition(single_time, positions));

      }

      // Button B6 - Grav Comp Mode
      if (mobile->getButtonDiff(6) == util::MobileIO::ButtonState::ToOn) {
        // cancel any goal that is set, returning arm into gravComp mode
        arm -> cancelGoal();
      }

      // Button B8 - End Demo
      if (mobile->getButtonDiff(8) == util::MobileIO::ButtonState::ToOn) {
        // Clear MobileIO text
        mobile->resetUI();
        return 1;
      }
    }

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile->clearText();

  return 0;
}



