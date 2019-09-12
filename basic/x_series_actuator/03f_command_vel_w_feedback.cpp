/**
 * Send velocity commands based on feedback from a module's internal gyro,
 * and log in the background.
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

  //// Closed-loop controller (velocity)
  GroupCommand group_command(group->size());
  GroupFeedback group_feedback(group->size());
  
  // Start logging in the background
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 0;
  }

  std::cout << "  Move the module to make the output move..." << std::endl;

  double duration = 15;               // [sec]
  auto start = std::chrono::system_clock::now();

  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  while (t.count() < duration) {
    group->getNextFeedback(group_feedback);
    t = std::chrono::system_clock::now() - start;

    // Command a velocity that counters the measured angular velocity around the
    // z-axis (same axis as the output)
    auto gyro_z = group_feedback.getGyro().col(2);
    group_command.setVelocity(-gyro_z);
    group->sendCommand(group_command);
  }

  // Stop logging
  auto log_file = group->stopLog();

  return 0;
}
