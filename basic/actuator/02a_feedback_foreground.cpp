#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::hebiChartsRunApplication(run, argc, argv);
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

  std::vector<double> y;
  y.resize(3, 0);
  std::vector<std::string> x_labels = {"X","Y","Z"};
  std::vector<double> x_ticks = {0.0,1.0,2.0};
  
  auto chart = hebi::charts::Chart::create();
  chart->getAxisY()->setLimits(-3.14, 3.14);
  // TODO:
  //chart->getAxisX()->setNames("X", "Y", "Z");
  //plt::xticks(x_ticks,x_labels);

  auto chart_data = chart->addBars("X/Y/Z", x_ticks.data(), y.data(), 3);
  chart->show();
  for (size_t i = 0; i < 50; ++i)
  { 
    if (group->getNextFeedback(group_fbk))
    {
      // Note -- can also retrieve individual module feedback; see API docs.
      // E.g., `group_fbk[0]` is the feedback from the first module.
      auto gyro = group_fbk.getGyro();
      y = { gyro(0,0), gyro(0,1), gyro(0,2) };
     
      //plot the feedback
      chart_data->setData(x_ticks.data(), y.data(), 3);
    }
  }

  hebi::charts::ChartFramework::waitUntilStagesClosed();

  return 0;
}