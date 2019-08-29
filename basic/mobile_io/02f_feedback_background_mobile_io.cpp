/*
 * Get feedback from a mobile io module and plot it live
 * 
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * HEBI Robotics
 * August 2019
 */

#include <chrono>
#include <iostream>
#include <thread>
 
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "util/plot_functions.h"

namespace plt = matplotlibcpp;

using namespace hebi;

int main() {

  // Find your module on the network 
  Lookup lookup;
  std::string family_name("HEBI");
  std::string module_name("Mobile IO");
  auto group = lookup.getGroupFromNames({family_name}, {module_name});

  // Confirm the module is found before preceding
  if (!group) {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  // Set the feedback frequency. 
  // This is by default "100"; setting this to 5 here allows the console output
  // to be more reasonable.
  group -> setFeedbackFrequencyHz(5);

  // Add a callback to react to feedback received on a background thread
  // Note: We use a C++11 "lambda function" here to pass in a function pointer,
  // but you can also pass in a C-style function pointer with the signature:
  //      void func(const hebi::GroupFeedback& group_fbk);
  std::vector<double> y;
  group->addFeedbackHandler([&y](const GroupFeedback& group_fbk) 
  {
    auto gyro = group_fbk.getGyro();
    y = {gyro(0,0), gyro(0,1), gyro(0,2)};

    // Plot the feedback
    std::vector<std::string> x_labels = {"X", "Y", "Z"};
    std::vector<double> x_ticks = {0.0, 1.0, 2.0};
    plt::clf();
    plt::ylim(-3.14, 3.14);
    plt::xticks(x_ticks, x_labels);
    plt::xlabel("Axis");
    plt::ylabel("Angular Velocity (rad/s)");
    plt::bar(y);
    plt::pause(0.01);
  });

  // Wait 10 seconds, and then stop and clear threads
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  group -> clearFeedbackHandlers();

  return 0;
}

