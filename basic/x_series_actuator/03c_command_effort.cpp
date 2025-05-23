/**
 * Send effort commands and log in the background.
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
#include "log_file.hpp"
#include "util/plot_functions.h"

namespace plt = matplotlibcpp;

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

  //// Open-loop controller (effort)

  // The command struct has fields for various commands and settings; for the
  // actuator, we will primarily use position, velocity, and effort.
  //
  // Fields that are not filled in will be ignored when sending.
  GroupCommand group_command(group->size());
  // GroupCommand uses Eigen types for data interchange
  Eigen::VectorXd efforts(1);
  // Allocate feedback
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
  double amp = 1.0;                   // [Nm]

  double duration = 10;               // [sec]
  auto start = std::chrono::system_clock::now();

  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  while (t.count() < duration) {
    // Even though we don't use the feedback, getting feedback conveniently
    // limits the loop rate to the feedback frequency
    group->getNextFeedback(group_feedback);

    // Update effort set point
    t = std::chrono::system_clock::now() - start;
    efforts[0] = amp * std::sin(freq * t.count());
    group_command.setEffort(efforts);
    group->sendCommand(group_command);
  }

  // Stop logging
  auto log_file = group->stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  //plot logged effort
  std::vector<std::vector<double>> eff;
  eff.resize(group->size());
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    for(size_t i = 0; i < group->size(); i++){
      eff[i].push_back(fbk.getEffort()[i]);
    }
  }
  for(size_t i = 0; i < group->size(); i++){
    plt::plot(eff[i]);
  }
  plt::show();

  return 0;
}
