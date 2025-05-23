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

  std::vector<float> pin_values(8); // we know we have 8 pins
  std::vector<std::string> x_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
  std::vector<double> x_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  // Start logging (you can also specify log file name as second parameter)
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  for (size_t i = 0; i < 50; ++i)
  {
    if (group -> getNextFeedback(group_fbk))
    {
     // Obtain I/O feedback from the groupFeedback object 
     auto& pin_data = group_fbk[0].io();

      // Analog Feedback
      // In this case, we gather only the values for A pins
      for (size_t i = 0; i < 8; ++i)
      {
        // we check pins i+1 because the pins are numbered 1-8
        if (pin_data.a().hasFloat(i+1)) {
          pin_values[i] = pin_data.a().getFloat(i+1);
        } else {
          pin_values[i] = (float) pin_data.a().getInt(i+1);
        }
      }

      // Now we plot the collected feedback
      plt::clf();
      plt::ylim(-1, 1);
      plt::xticks(x_ticks, x_labels);
      plt::title("IO Board Feedback from IO pins");
      plt::xlabel("Pin Number");
      plt::ylabel("[-1 to 1]");
      plt::bar(pin_values);
      plt::pause(0.01);
    }
  }

  // Stop logging
  std::shared_ptr<LogFile> log_file = group -> stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  group -> clearFeedbackHandlers();
  return 0;
}

