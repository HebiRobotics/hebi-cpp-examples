#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**)
{
  // Get a group
  Lookup lookup;
  auto group = lookup.getGroupFromNames({ "Test Family" }, { "Test Actuator" });
  if (!group)
  {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  // This is by default "100"; setting this to 5 here allows the console output
  // to be more reasonable.
  group->setFeedbackFrequencyHz(5);

  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::Window window;
    auto chart = window.addChart();
    chart.setTitle("Gyro Feedback");
    chart.getAxisX().setName("time (s)");
    chart.getAxisY().setName("rad/s");
    chart.getAxisY().setLimits(-3.14, 3.14);
    auto x_data = chart.addLine("X", {}, {});
    auto y_data = chart.addLine("Y", {}, {});
    auto z_data = chart.addLine("Z", {}, {});
    window.show();
    // Add a callback to react to feedback received on a background thread
    // Note: We use a C++11 "lambda function" here to pass in a function pointer,
    // but you can also pass in a C-style function pointer with the signature:
    //   void func(const hebi::GroupFeedback& group_fbk);
    group->addFeedbackHandler([&x_data, &y_data, &z_data](const GroupFeedback& group_fbk)
    {
      double t = group_fbk.getTime();
      auto gyro = group_fbk.getGyro();
      x_data.addPoint(t, gyro(0, 0));
      y_data.addPoint(t, gyro(0, 1));
      z_data.addPoint(t, gyro(0, 2));
    });

    // Wait for 10 seconds, and then stop.
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    group->clearFeedbackHandlers();

    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}