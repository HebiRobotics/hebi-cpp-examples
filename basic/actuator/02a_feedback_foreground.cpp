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
  auto group = lookup.getGroupFromNames({ "Family Name" }, { "Group Name" });
  if (!group)
  {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  // This is by default "100"; setting this to 5 here allows the console output
  // to be more reasonable.
  group->setFeedbackFrequencyHz(5);

  // Retrieve feedback with a blocking call to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run for
  // about 10 seconds here
  GroupFeedback group_fbk(group->size());

  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::GridWindow window;
    auto chart = window.addLineChart();
    chart.setTitle("Gyro Feedback");
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
        // Note -- can also retrieve individual module feedback; see API docs.
        // E.g., `group_fbk[0]` is the feedback from the first module.
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