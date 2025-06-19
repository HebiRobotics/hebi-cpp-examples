/*
 * Get feedback from a mobile io module and plot it live
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
  group->setFeedbackFrequencyHz(5);

  // Add a callback to react to feedback received on a background thread
  // Note: We use a C++11 "lambda function" here to pass in a function pointer,
  // but you can also pass in a C-style function pointer with the signature:
  //      void func(const hebi::GroupFeedback& group_fbk);
  std::vector<double> y;
  if (hebi::charts::framework::isLoaded()) {
    std::vector<double> y;
    y.resize(3,0);
    std::vector<std::string> x_labels = {"X", "Y", "Z"};
    std::vector<double> x_ticks = {0.0, 1.0, 2.0};

    hebi::charts::Chart chart;
    chart.setTitle("Mobile I/O Gyro Feedback");
    chart.getAxisY().setLimits(-3.14, 3.14);
    chart.getAxisX().setName("Axis");
    chart.getAxisY().setName("Angular Velocity (rad/s)");
    // TODO:
    //chart.getAxisX().setNames(x_labels);
    //chart.getAxisX().setTicks(x_ticks);

    auto chart_data = chart.addBars("X/Y/Z", x_ticks, y);
    group->addFeedbackHandler([&y, &x_ticks, &chart_data](const GroupFeedback& group_fbk) 
    {
      auto gyro = group_fbk.getGyro();
      y = {gyro(0,0), gyro(0,1), gyro(0,2)};

      // Plot the feedback
      chart_data.setData(x_ticks, y);
    });

    // Wait 10 seconds, and then stop and clear threads
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    group->clearFeedbackHandlers();
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

