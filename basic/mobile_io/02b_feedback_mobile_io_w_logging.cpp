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

  hebi::charts::Window window;
  auto chart = window.addChart();
  chart.setTitle("Mobile I/O Input Feedback");
  chart.getAxisY().setLimits(-1, 1);
  chart.getAxisX().setName("timestep");
  chart.getAxisY().setName("[-1 to 1]");

  std::array<hebi::charts::Dataset, 8> button_chart_data = {
    chart.addLine("Button 1", {}, {}),
    chart.addLine("Button 2", {}, {}),
    chart.addLine("Button 3", {}, {}),
    chart.addLine("Button 4", {}, {}),
    chart.addLine("Button 5", {}, {}),
    chart.addLine("Button 6", {}, {}),
    chart.addLine("Button 7", {}, {}),
    chart.addLine("Button 8", {}, {})
  };
  std::array<hebi::charts::Dataset, 8> slider_chart_data = {
    chart.addLine("Slider 1", {}, {}),
    chart.addLine("Slider 2", {}, {}),
    chart.addLine("Slider 3", {}, {}),
    chart.addLine("Slider 4", {}, {}),
    chart.addLine("Slider 5", {}, {}),
    chart.addLine("Slider 6", {}, {}),
    chart.addLine("Slider 7", {}, {}),
    chart.addLine("Slider 8", {}, {})
  };
  window.show();
  for (size_t i = 0; i < 50; ++i)
  {
    if (group->getNextFeedback(group_fbk))
    {
      // Obtain feedback for a singular module from the groupFeedback object
      auto& io_data = group_fbk[0].io();

      // Digital Feedback (Buttons) 
      // We can safely assume that all buttons return an int value
      // Store as double for interop with the plotting library
      // Note -- here and below we check pins i+1 because the pins are numbered 1-8, not 0-7
      for (size_t j = 0; j < 8; ++j)
        button_chart_data[j].addPoint(i, static_cast<double>(io_data.b().getInt(j + 1)));

      // Analog Feedback (Sliders) 
      // We expect float values, but may recieve an int in certain cases.
      // As such, we convert any ints we encounter back to float
      for (size_t j = 0; j < 8; ++j)
      {
        double slider_value = 0;
        if (io_data.a().hasFloat(j + 1)) {
          slider_value = io_data.a().getFloat(j + 1);
        } else {
          slider_value = static_cast<double>(io_data.a().getInt(j + 1));
        }
        slider_chart_data[j].addPoint(i, slider_value);
      }
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

