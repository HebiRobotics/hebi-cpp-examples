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

  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::GridWindow window;
    auto chart = window.addLineChart();
    chart.setTitle("Mobile I/O Gyro Feedback");
    chart.getAxisX().setName("timestep");
    chart.getAxisY().setName("rad/s");
    chart.getAxisY().setLimits(-3.14, 3.14);

    auto x_data = chart.addLine("X", {}, {});
    auto y_data = chart.addLine("Y", {}, {});
    auto z_data = chart.addLine("Z", {}, {});
    window.show();
    for (size_t i = 0; i < 50; ++i)
    {
      if (group->getNextFeedback(group_fbk))
      {
        // Use the Hebi API to extract gyro data from the module
        auto gyro = group_fbk.getGyro();
        x_data.addPoint(i, gyro(0, 0));
        y_data.addPoint(i, gyro(0, 1));
        z_data.addPoint(i, gyro(0, 2));
      }
    }
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

