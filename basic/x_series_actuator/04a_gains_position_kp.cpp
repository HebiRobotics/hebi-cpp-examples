/**
 * Change the P-gain of the position controller on a module while commanding
 * a step change in position
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * October 2018
 */

#include <iostream>
#include <chrono>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
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
  auto group = lookup.getGroupFromNames({"Test Family"}, {"Test Actuator" });

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  // The command struct has fields for gains in addition to the basic position,
  // velocity, and effort commands.
  GroupCommand gain_command(group->size());
  // We add a separate command object for our step command, so we are not
  // sending a gain command in each position command.
  GroupCommand position_command(group->size());
  GroupFeedback feedback(group->size());
 
  // Start logging in the background
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  // Parameters for step function
  double step_period = 1.0;                  // sec
  double step_amplitude = M_PI / 8.0;        // rad
  double step_duration = step_period * 4.0;  // the 4.0 multiplier gives 2 full cycles

  // Make the position controller gradually stiffer.
  std::vector<double> new_position_kp_gains { 0.2, 0.5, 10.0 };

  for (auto new_gain : new_position_kp_gains) {
    // Update command and send to the actuator
    for (int i = 0; i < group->size(); ++i)
      gain_command[i].settings().actuator().positionGains().kP().set(new_gain);
    if (!group->sendCommandWithAcknowledgement(gain_command)) {
      std::cout << "Did not get acknowledgement from module when sending gains; check connection." << std::endl;
      return -1;
    }
    std::cout << "Set position kP gain to: " << new_gain << std::endl;

    // Command a step function to show actuator response
    auto start = std::chrono::system_clock::now();
    std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
    Eigen::VectorXd position_vector(group->size());
    while (t.count() < step_duration) {
      // Get feedback to limits the loop rate
      group->getNextFeedback(feedback);

      // Calculate position step function, and update command
      t = std::chrono::system_clock::now() - start;
      double position = std::fmod(t.count(), 2.0 * step_period) > step_period ?
        step_amplitude : 0.0;
      position_vector.setConstant(position);
      position_command.setPosition(position_vector);

      // Send command
      group->sendCommand(position_command);
    }
  }

  // Stop logging
  auto log_file = group->stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  // plot logged position
  std::vector<std::vector<double>> pos;
  pos.resize(group->size());
  double t0{};
  std::vector<double> times;
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    for(size_t i = 0; i < group->size(); i++){
      pos[i].push_back(fbk.getPosition()[i]);
    }
    if (t0 == 0)
      t0 = fbk.getTime();
    times.push_back(fbk.getTime() - t0);
  }
  if (hebi::charts::framework::isLoaded()) {
    hebi::charts::Chart chart;
    for(size_t i = 0; i < group->size(); i++){
      auto title =(std::string("module ") + std::to_string(i));
      chart.addLine(title, times, pos[i]);
    }
    chart.show();

    hebi::charts::framework::waitUntilWindowsClosed();
  }
  return 0;
}