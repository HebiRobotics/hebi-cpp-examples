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
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**) {

  if (!hebi::charts::lib::isAvailable()) {
    std::cout << "Plotting library not found; cannot continue!" << std::endl;
    return 1;
  }

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
  group->setFeedbackFrequencyHz(5);

  // Retrieve feedback with a blocking all to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run 
  // for about 10 seconds here
  GroupFeedback group_fbk(group->size());

  std::cout << "\n Drag the Sliders and press some buttons on the app screen!" 
            << std::endl;

  // Start logging (you can also specify log file name as second parameter)
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  std::vector<double> buttons;
  buttons.resize(8, 0);// we know we have 8 pins
  std::vector<double> sliders;
  sliders.resize(8, 0);// we know we have 8 pins
  std::vector<std::string> x_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
  std::vector<double> x_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  hebi::charts::Chart chart;
  chart.setTitle("Mobile I/O Input Feedback");
  chart.getAxisY().setLimits(-1, 1);
  chart.getAxisX().setName("Digital Inputs and Analog Inputs");
  chart.getAxisY().setName("[-1 to 1]");
  // TODO:
  //chart.getAxisX().setNames(x_labels);
  //chart.getAxisX().setTicks(x_ticks);

  auto button_chart_data = chart.addBars("X/Y/Z", x_ticks, buttons);
  auto slider_chart_data = chart.addBars("X/Y/Z", x_ticks, sliders);
  chart.show();
  for (size_t i = 0; i < 50; ++i)
  {
    if (group->getNextFeedback(group_fbk))
    {
      // Obtain feedback for a singular module from the groupFeedback object
      auto& buttons_data = group_fbk[0].io();

      // Digital Feedback (Buttons) 
      // We can safely assume that all buttons return an int value
      // Store as double for interop with the plotting library
      buttons = {static_cast<double>(buttons_data.b().getInt(1)),
                  static_cast<double>(buttons_data.b().getInt(2)),
                  static_cast<double>(buttons_data.b().getInt(3)),
                  static_cast<double>(buttons_data.b().getInt(4)),
                  static_cast<double>(buttons_data.b().getInt(5)),
                  static_cast<double>(buttons_data.b().getInt(6)),
                  static_cast<double>(buttons_data.b().getInt(7)),
                  static_cast<double>(buttons_data.b().getInt(8))};

      // Analog Feedback (Sliders) 
      // We expect float values, but may recieve an int in certain cases.
      // As such, we convert any ints we encounter back to float
      for (size_t i = 0; i < 8; ++i)
      {
        // we check pins i+1 because the pins are numbered 1-8, not 0-7
        if (buttons_data.a().hasFloat(i+1)) {
          sliders[i] = buttons_data.a().getFloat(i+1);
        } else {
          sliders[i] = static_cast<double>(buttons_data.a().getInt(i+1));
        }
      }

      // Now we plot the collected feedback
      button_chart_data.setData(x_ticks, buttons);
      slider_chart_data.setData(x_ticks, sliders);
    }
  }

  // Stop logging
  std::shared_ptr<LogFile> log_file = group -> stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  hebi::charts::framework::waitUntilWindowsClosed();

  return 0;
}

