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
    std::vector<double> y;
    y.resize(3, 0);
    std::vector<std::string> x_labels = {"X","Y","Z"};
    std::vector<double> x_ticks = {0.0,1.0,2.0};
  
    hebi::charts::Chart chart;
    chart.getAxisY().setLimits(-3.14, 3.14);
    // TODDO:
    //chart->getAxisX()->setNames("X", "Y", "Z");
    //plt::xticks(x_ticks,x_labels);

    auto chart_data = chart.addBars("X/Y/Z", x_ticks, y);
    chart.show();
    // Add a callback to react to feedback received on a background thread
    // Note: We use a C++11 "lambda function" here to pass in a function pointer,
    // but you can also pass in a C-style function pointer with the signature:
    //   void func(const hebi::GroupFeedback& group_fbk);
    group->addFeedbackHandler([&x_ticks, &y, &chart_data](const GroupFeedback& group_fbk)
    {
      auto gyro = group_fbk.getGyro();
      y = { gyro(0,0), gyro(0,1), gyro(0,2) };

      //plot the feedback
      chart_data.setData(x_ticks, y);
    });

    // Wait for 10 seconds, and then stop.
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    group->clearFeedbackHandlers();

    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}