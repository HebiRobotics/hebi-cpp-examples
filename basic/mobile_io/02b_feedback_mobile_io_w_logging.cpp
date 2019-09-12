/*
 * Get feedback from a singular mobile io module and plot it live.
 * Generate and save a log file for the 10 seconds this is run
 *
 * This example uses blocking. For a non-blocking example, please check
 * example 02f_feedback_background_mobile_io
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
  // in your group.
  Lookup lookup;
  std::string family_name("HEBI");
  std::string module_name("Mobile IO");
  auto group = lookup.getGroupFromNames({family_name}, {module_name});

  // Confirm the module is found before proceeding
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

  std::vector<int64_t> buttons;
  std::vector<float> sliders(8); // we know we have 8 pins
  std::vector<std::string> x_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
  std::vector<double> x_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  std::cout << "\n Drag the Sliders and press some buttons on the app screen!" 
            << std::endl;

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
      // Obtain feedback for a singular module from the groupFeedback object
      auto& buttons_data = group_fbk[0].io();

      // Digital Feedback (Buttons) 
      // We can safely assume that all buttons return an int value
      buttons = {buttons_data.b().getInt(1),
                 buttons_data.b().getInt(2),
                 buttons_data.b().getInt(3),
                 buttons_data.b().getInt(4),
                 buttons_data.b().getInt(5),
                 buttons_data.b().getInt(6),
                 buttons_data.b().getInt(7),
                 buttons_data.b().getInt(8)};

      // Analog Feedback (Sliders) 
      // We expect float values, but may recieve an int in certain cases.
      // As such, we convert any ints we encounter back to float
      for (size_t i = 0; i < 8; ++i)
      {
        // we check pins i+1 because the pins are numbered 1-8, not 0-7
        if (buttons_data.a().hasFloat(i+1)) {
          sliders[i] = buttons_data.a().getFloat(i+1);
        } else {
          sliders[i] = (float)buttons_data.a().getInt(i+1);
        }
      }

      // Now we plot the collected feedback
      plt::clf();
      plt::ylim(-1, 1);
      plt::xticks(x_ticks, x_labels);
      plt::xlabel("Digital Inputs and Analog Inputs");
      plt::ylabel("[-1 to 1]"); 
      plt::bar(sliders);
      plt::bar(buttons);
      plt::pause(0.01);
    }
  }

  // Stop logging
  std::shared_ptr<LogFile> log_file = group -> stopLog();

  group -> clearFeedbackHandlers();
  return 0;
}

