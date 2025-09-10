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
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**) {
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

  // Start logging (you can also specify log file name as second parameter)
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::GridWindow window;
    auto chart = window.addLineChart();
    chart.setTitle("IO Board Feedback from IO pins");
    chart.getAxisY().setLimits(0, 5);
    chart.getAxisX().setName("timestep");
    chart.getAxisY().setName("[0 to 5]");

    std::array<hebi::charts::Line, 8> pin_chart_data = {
      chart.addLine("Pin 1", {}, {}),
      chart.addLine("Pin 2", {}, {}),
      chart.addLine("Pin 3", {}, {}),
      chart.addLine("Pin 4", {}, {}),
      chart.addLine("Pin 5", {}, {}),
      chart.addLine("Pin 6", {}, {}),
      chart.addLine("Pin 7", {}, {}),
      chart.addLine("Pin 8", {}, {}),
    };
    window.show();
    for (size_t i = 0; i < 50; ++i)
    {
      if (group -> getNextFeedback(group_fbk))
      {
        // Obtain I/O feedback from the groupFeedback object 
        auto& pin_data = group_fbk[0].io();

        // Analog Feedback
        // In this case, we gather only the values for A pins
        for (size_t pin = 0; pin < 8; ++pin)
        {
          // we check pins i+1 because the pins are numbered 1-8
          if (pin_data.a().hasFloat(pin+1)) {
            pin_chart_data[pin].addPoint(i, pin_data.a().getFloat(pin+1));
          } else {
            pin_chart_data[pin].addPoint(i, static_cast<double>(pin_data.a().getInt(pin+1)));
          }
        }
      }
    }

    hebi::charts::framework::waitUntilWindowsClosed();
  }

  // Stop logging
  std::shared_ptr<LogFile> log_file = group -> stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  return 0;
}

