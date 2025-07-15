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
  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::Chart chart;
    chart.setTitle("Mobile I/O Gyro Feedback");
    chart.getAxisX().setName("time (s)");
    chart.getAxisY().setName("rad/s");
    chart.getAxisY().setLimits(-3.14, 3.14);

    auto x_data = chart.addLine("X", {}, {});
    auto y_data = chart.addLine("Y", {}, {});
    auto z_data = chart.addLine("Z", {}, {});
    group->addFeedbackHandler([&x_data, &y_data, &z_data](const GroupFeedback& group_fbk) 
    {
      double t = group_fbk.getTime();
      auto gyro = group_fbk.getGyro();
      x_data.addPoint(t, gyro(0, 0));
      y_data.addPoint(t, gyro(0, 1));
      z_data.addPoint(t, gyro(0, 2));
    });

    // Wait 10 seconds, and then stop and clear threads
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    group->clearFeedbackHandlers();
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

