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
  group->setFeedbackFrequencyHz(5);

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
    chart.setTitle("IO Board Gyro Feedback");
    chart.getAxisX().setName("timestep");
    chart.getAxisY().setName("rad/s");
    chart.getAxisY().setLimits(-3.14, 3.14);

    auto x_data = chart.addLine("X", {}, {});
    auto y_data = chart.addLine("Y", {}, {});
    auto z_data = chart.addLine("Z", {}, {});
    window.show();
    for (size_t i = 0; i < 50; ++i)
    {
      if (group -> getNextFeedback(group_fbk))
      {
        // Obtain gryo feedback from the groupFeedback object
        auto gyro = group_fbk.getGyro();
        x_data.addPoint(i, gyro(0, 0));
        y_data.addPoint(i, gyro(0, 1));
        z_data.addPoint(i, gyro(0, 2));
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

