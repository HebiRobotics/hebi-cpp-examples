/*
 * Simultaneously read analog, digital inputs, and gyro feedback and 
 * visualize online
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
  // Find your mobile device on the network 
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

  // Retrieve feedback with a blocking all to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run 
  // for about 10 seconds here
  GroupFeedback group_fbk(group->size());

  std::vector<int64_t> buttons;
  std::vector<float> sliders(8); // we know we have 8 pins
  std::vector<std::string> x1_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
  std::vector<double> x1_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  std::vector<double> gyro_data;
  std::vector<std::string> x2_labels = {"X", "Y", "Z"};
  std::vector<double> x2_ticks = {0.0, 1.0, 2.0};

  std::cout << "\n Drag the Sliders, press some buttons, and move the device..." 
            << std::endl;

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
        // we check pins i+1 because the pins are numbered 1-8
        if (buttons_data.a().hasFloat(i+1)) {
          sliders[i] = buttons_data.a().getFloat(i+1);
        } else {
          sliders[i] = (float)buttons_data.a().getInt(i+1);
        }
      }

      // Gyro Feedback
      auto gyro = group_fbk.getGyro();
      gyro_data = {gyro(0,0), gyro(0,1), gyro(0,2)};

      // Now we plot the collected data
      plt::clf();
      plt::subplot(2, 1, 1); // io feedback
        plt::ylim(-1, 1);
        plt::xticks(x1_ticks, x1_labels);
        plt::xlabel("Digital Inputs and Analog Inputs");
        plt::ylabel("[-1 to 1]");
        plt::bar(sliders);
        plt::bar(buttons);
      plt::subplot(2, 1, 2); // gyro feedback
        plt::ylim(-3.14, 3.14);
        plt::xticks(x2_ticks, x2_labels);
        plt::xlabel("Axis");
        plt::ylabel("Angular Velocity (rad/s)");
        plt::bar(gyro_data);
      plt::pause(0.01);
    }
  }

  group -> clearFeedbackHandlers();
  return 0;
}

