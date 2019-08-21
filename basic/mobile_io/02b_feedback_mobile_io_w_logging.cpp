/*
 * Get feedback from a singular mobile io module and plot it live.
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
#include "plot_functions.h"

namespace plt = matplotlibcpp;

using namespace hebi;


int main() {

  // Find your module on the network 
  // You can also plot feedback from multiple modules by including multiple modules
  // in your group. Look at example 01c on how to do that.
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
  // const Feedback mobile_fbk();


  std::vector<double> y;
  std::vector<int64_t> buttons;
  std::vector<float> sliders;
  std::vector<std::string> x_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
  std::vector<double> x_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};


  // Start logging (can also specify log file name as second parameter)
  std::string full_log_path = group -> startLog("./logs/");

  std::cout << full_log_path << std::endl;

  for (size_t i = 0; i < 50; ++i)
  {
    if (group -> getNextFeedback(group_fbk))
    {
      //auto& mobile_fbk = group_fbk[0];
      // auto& buttons_data = mobile_fbk.io();
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
      // Expect float values, but may recieve an int in certain cases.
      // As such, we convert any ints we encounter back to float
      sliders.clear();
      for (size_t i = 0; i < 8; ++i)
      {
        // sliders.push_back(buttons_data.a().hasFloat(i+1) ? buttons_data.a().getFloat(i+1) : (float)buttons_data.a().getInt(i+1));
      
        if (buttons_data.a().hasFloat(i+1)) {
          sliders.push_back(buttons_data.a().getFloat(i+1));
        } else {
          sliders.push_back((float)buttons_data.a().getInt(i+1));
        }

      }


      // Now we plot the collected feedback
      plt::clf();
      plt::ylim(-3.14, 3.14);
      plt::xticks(x_ticks, x_labels);
      plt::xlabel("Axis");
      plt::ylabel("Angular Velocity (rad/s)");
      // plt::bar
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

