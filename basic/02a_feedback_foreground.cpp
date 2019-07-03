#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "matplotlibcpp.h"

using namespace hebi;

namespace plt = matplotlibcpp;

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
  std::vector<double> x;
  std::vector<std::string> labels = {"Position","Velocity","Effort"};
  std::vector<double> label_spacing = {0,1.0,2.0};
  double y_min = -3.14;
  double y_max = 3.14;
  for (size_t i = 0; i < 50; ++i)
  { 
    if (group->getNextFeedback(group_fbk))
    {
      std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
      double position = static_cast<double>(group_fbk.getPosition()[0]);
      double velocity = static_cast<double>(group_fbk.getVelocity()[0]);
      double effort = static_cast<double>(group_fbk.getEffort()[0]);
      y.clear();
      x.clear();
      y.push_back(position);
      y.push_back(velocity);
      y.push_back(effort);
      plt::clf();
      plt::xticks(label_spacing,labels);
            if ((position > y_max || velocity > y_max || effort > y_max)||(position < y_min || velocity < y_min || effort < y_min)){
         y_min *= 2;
	 y_max *= 2;
      }
      if ((position < y_max/2 && velocity < y_max/2 && effort < y_max/2) && (position > y_min/2 && velocity > y_min/2 && effort > y_min/2)) {
        y_max /= 2;
	y_min /= 2;
      }
      std::vector<double> y_ticks;
      for(double j = 0; j < y_max; j += .5){
        y_ticks.push_back(j);
      }
      for(double j = 0; j > y_min; j -= .5){
        y_ticks.push_back(j);
      }
      plt::yticks(y_ticks);

      plt::grid(true);
      plt::ylim(y_min, y_max); 
      plt::bar(y, "black", "-",.5);
      plt::pause(0.01);
      // Note -- can also retrieve individual module feedback; see API docs.
      // E.g., `group_fbk[0]` is the feedback from the first module.
    }
  }

  group->clearFeedbackHandlers();

  return 0;
}
