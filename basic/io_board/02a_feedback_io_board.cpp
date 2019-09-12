/*
 * Get feedback from a singular io_board and plot it live.
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
  // You can also plot feedback from multiple modules by including multiple modules
  // in your group. Look at example 01c on how to do that.
  Lookup lookup;
  std::string family_name("HEBI");
  std::string module_name("IO Board");
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

  // Retrieve feedback with a blocking all to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run 
  // for about 10 seconds here
  GroupFeedback group_fbk(group->size());

  std::vector<double> gyro_data;
  std::vector<std::string> x_labels = {"X", "Y", "Z"};
  std::vector<double> x_ticks = {0.0, 1.0, 2.0};

  // Start logging (you can also specify log file name as second parameter)
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 0;
  }

  for (size_t i = 0; i < 50; ++i)
  {
    if (group -> getNextFeedback(group_fbk))
    {
      // Obtain gryo feedback from the groupFeedback object
      auto gyro = group_fbk.getGyro();
      gyro_data = {gyro(0,0), gyro(0,1), gyro(0,2)};

      // Now we plot the collected feedback
      plt::clf();
      plt::ylim(-3.14, 3.14);
      plt::xticks(x_ticks, x_labels);
      plt::title("IO Board Gyro Feedback");
      plt::xlabel("Axis");
      plt::ylabel("Angular Velocity (rad/s)");
      plt::bar(gyro_data);
      plt::pause(0.01);
    }
  }

  // Stop logging
  std::shared_ptr<LogFile> log_file = group -> stopLog();

  group -> clearFeedbackHandlers();
  return 0;
}

