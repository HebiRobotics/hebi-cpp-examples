/**
 * Send simultaneous position and velocity commands and log in the background.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * September 2018
 */

#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"

using namespace hebi;

int main() {
  // Get group
  Lookup lookup;
  auto group = lookup.getGroupFromNames({"Test Family"}, {"Test Actuator" });

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  //// Open-loop controller (position + velocity)

  // Fields that are not filled in will be ignored when sending.
  GroupCommand group_command(group->size());
  Eigen::VectorXd positions(1);
  Eigen::VectorXd velocities(1);
  GroupFeedback group_feedback(group->size());
  
  // Start logging in the background
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  // Parameters for sin/cos function
  double freq_hz = 0.5;               // [Hz]
  double freq = freq_hz * 2.0 * M_PI; // [rad / sec]
  double amp = M_PI / 4.0;            // [rad] (45 degrees)

  double duration = 10;               // [sec]
  auto start = std::chrono::system_clock::now();

  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  while (t.count() < duration) {
    // Even though we don't use the feedback, getting feedback conveniently
    // limits the loop rate to the feedback frequency
    group->getNextFeedback(group_feedback);

    t = std::chrono::system_clock::now() - start;

    // Position command
    positions[0] = amp * std::sin(freq * t.count());

    // Velocity command (time derivative of position)
    velocities[0] = freq * amp * std::cos(freq * t.count());

    // Update set points
    group_command.setPosition(positions);
    group_command.setVelocity(velocities);
    group->sendCommand(group_command);
  }

  // Stop logging
  auto log_file = group->stopLog();

  return 0;
}
