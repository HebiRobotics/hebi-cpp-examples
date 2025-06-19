/**
 * Send velocity commands and log in the background.
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
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**) {
  // Get group
  Lookup lookup;
  auto group = lookup.getGroupFromNames({"Test Family"}, {"Test Actuator"});

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  //// Open-loop controller (velocity)

  // The command struct has fields for various commands and settings; for the
  // actuator, we will primarily use position, velocity, and effort.
  //
  // Fields that are not filled in will be ignored when sending.
  GroupCommand group_command(group->size());
  // GroupCommand uses Eigen types for data interchange
  Eigen::VectorXd velocities(1);
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
  double amp = 1.0;                   // [rad / sec]

  double duration = 10;               // [sec]
  auto start = std::chrono::system_clock::now();

  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  while (t.count() < duration) {
    // Even though we don't use the feedback, getting feedback conveniently
    // limits the loop rate to the feedback frequency
    group->getNextFeedback(group_feedback);

    // Update velocity set point
    t = std::chrono::system_clock::now() - start;
    velocities[0] = amp * std::sin(freq * t.count());
    group_command.setVelocity(velocities);
    group->sendCommand(group_command);
  }

  // Stop logging
  auto log_file = group->stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  //plot logged velocity
  std::vector<std::vector<double>> vel;
  vel.resize(group->size());
  double t0{};
  std::vector<double> times;
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    for(size_t i = 0; i < group->size(); i++){
      vel[i].push_back(fbk.getVelocity()[i]);
    }
    if (t0 == 0)
      t0 = fbk.getTime();
    times.push_back(fbk.getTime() - t0);
  }
  if (hebi::charts::framework::isLoaded()) {
    hebi::charts::Chart chart;
    for(size_t i = 0; i < group->size(); i++){
      auto title = (std::string("module ") + std::to_string(i));
      chart.addLine(title, times, vel[i]);
    }
    chart.show();

    hebi::charts::framework::waitUntilWindowsClosed();
  }
  return 0;
}