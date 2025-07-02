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
    std::vector<double> gyro_data;
    gyro_data.resize(3,0);
    std::vector<std::string> x_labels = {"X", "Y", "Z"};
    std::vector<double> x_ticks = {0.0, 1.0, 2.0};

    hebi::charts::Chart chart;
    chart.setTitle("IO Board Gyro Feedback");
    chart.getAxisY().setLimits(-3.14, 3.14);
    chart.getAxisX().setName("Axis");
    chart.getAxisY().setName("Angular Velocity (rad/s)");
    // TODO:
    //chart.getAxisX().setNames(x_labels);
    //chart.getAxisX().setTicks(x_ticks);

    auto chart_data = chart.addBars("X/Y/Z", x_ticks, gyro_data);
    chart.show();
    for (size_t i = 0; i < 50; ++i)
    {
      if (group -> getNextFeedback(group_fbk))
      {
        // Obtain gryo feedback from the groupFeedback object
        auto gyro = group_fbk.getGyro();
        gyro_data = {gyro(0,0), gyro(0,1), gyro(0,2)};

        // Now we plot the collected feedback
        chart_data.setData(x_ticks, gyro_data);
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

