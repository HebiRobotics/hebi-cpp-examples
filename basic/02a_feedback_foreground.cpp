#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "plot_functions.h"

namespace plt = matplotlibcpp;

using namespace hebi;


int main()
{
  // Get a group
  Lookup lookup;
  auto group = lookup.getGroupFromNames({ "HEBI" }, { "X-00147" });
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
  std::vector<std::string> x_labels = {"X","Y","Z"};
  std::vector<double> x_ticks = {0.0,1.0,2.0};

  for (size_t i = 0; i < 50; ++i)
  { 
    if (group->getNextFeedback(group_fbk))
    {
      auto gyro = group_fbk.getGyro();
      y = {gyro(0,0),gyro(0,1), gyro(0,2) };
      
      plt::clf();
      plt::ylim(-3.14, 3.14); 
      plt::xticks(x_ticks,x_labels);
      plt::bar(y);
      plt::pause(0.01);
      // Note -- can also retrieve individual module feedback; see API docs.
      // E.g., `group_fbk[0]` is the feedback from the first module.
    }
  }

  group->clearFeedbackHandlers();

  return 0;
}
