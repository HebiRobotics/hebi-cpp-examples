/**
 * Send position commands and log in the background.
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

  //// Open-loop controller (position)

  // The command struct has fields for various commands and settings; for the
  // actuator, we will primarily use position, velocity, and effort.
  //
  // Fields that are not filled in will be ignored when sending.
  GroupCommand group_command(group->size());
  // GroupCommand uses Eigen types for data interchange
  Eigen::VectorXd positions(1);
  // Allocate feedback
  GroupFeedback group_feedback(group->size());
  
  // Start logging in the background
  std::string log_path = group->startLog("logs");

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

  double duration = 2;               // [sec]
  auto start = std::chrono::system_clock::now();

  std::vector<double> pos;
  std::vector<double> times;
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  double t0{};
  while (t.count() < duration) {
    // Even though we don't use the feedback, getting feedback conveniently
    // limits the loop rate to the feedback frequency
    group->getNextFeedback(group_feedback);

    // Update position set point
    t = std::chrono::system_clock::now() - start;
    positions[0] = amp * std::sin(freq * t.count());
    group_command.setPosition(positions);
    group->sendCommand(group_command);
    pos.push_back(group_feedback.getPosition()[0]);
    if (t0 == 0)
      t0 = group_feedback.getTime();
    times.push_back(group_feedback.getTime() - t0);
  }

  // Stop logging
  auto log_file = group->stopLog();

  //plot the logged position data
  //std::vector<std::vector<double>> pos;
  //std::vector<double> pos;
  //std::vector<double> times;
  //pos.resize(group->size());
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    //for(size_t i = 0; i < group->size(); i++){
      //pos.push_back(fbk.getPosition()[i]);
    //}
    // TODO: assert size == 1?
    //pos.push_back(fbk.getPosition()[0]);
    //times.push_back(fbk.getTime());
    //std::cout << times.back() << " " << pos.back() << "\n";
  }
  plotting::FxRuntime::setTheme("Dracula");
  auto chart = plotting::Chart::create();
  chart->setTitle("03a_command_position");
  chart->show(); // ???
  // TODO: pull from name above?
  auto dataset = chart->addLine("Test Actuator", nullptr, nullptr, 0);
  dataset->addStyleClass("trace-solid");
  dataset->addStyleClass("color-magenta");
  dataset->setData(times.data(), pos.data(), pos.size());
  //for(size_t i = 0; i < group->size(); i++){
    //plt::plot(pos[i]);
  //}
  //plt::show();

  plotting::FxRuntime::waitUntilStagesClosed();

  return 0;
}
