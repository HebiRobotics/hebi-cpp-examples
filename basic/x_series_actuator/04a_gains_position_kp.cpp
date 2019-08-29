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

  // The command struct has fields for gains in addition to the basic position,
  // velocity, and effort commands.
  GroupCommand gain_command(group->size());
  // We add a separate command object for our step command, so we are not
  // sending a gain command in each position command.
  GroupCommand position_command(group->size());
  GroupFeedback feedback(group->size());
 
  // Start logging in the background
  group->startLog("logs");

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

  // plot logged position
  std::vector<std::vector<double>> pos;
  pos.resize(group->size());
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    for(size_t i = 0; i < group->size(); i++){
      pos[i].push_back(fbk.getPosition()[i]);
    }
  }
  for(size_t i = 0; i < group->size(); i++){
    plt::plot(pos[i]);
  }
  plt::show();
  return 0;
}
