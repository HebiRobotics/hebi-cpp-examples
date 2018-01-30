/**
 * This file demonstrates the ability to get led feedback from a group.
 */

#include "lookup.hpp"
#include "group_feedback.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

int main(int argc, char* argv[])
{
  // Try and get the requested group.
  std::shared_ptr<hebi::Group> group;
  {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames({"X5-4"}, {"X5-0000"});
    if (!group)
    {
      std::cout << "No group found!" << std::endl;
      return -1;
    }
  }

  int num_modules = group->size();

  // Create a group feedback object; this will be filled in during the request.
  hebi::GroupFeedback feedback(num_modules);

  // In a loop, send requests for feedback to the group and wait for responses.
  long timeout_ms = 1000;
  float period_s = 0.25f;
  for (int i = 0; i < 20; i++)
  {
    group->sendFeedbackRequest();
    if (group->getNextFeedback(feedback, timeout_ms))
    {
      for (size_t mod_idx = 0; mod_idx < feedback.size(); mod_idx++)
      {
        if (feedback[mod_idx].led().hasColor())
        {
          hebi::Color color = feedback[mod_idx].led().getColor();
          std::cout << color.getRed() << "-" << color.getGreen() << "-" << color.getBlue() << "  ";
        }
        std::cout << std::endl;
      }
    }
    else
      std::cout << "Received no feedback from group!" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period_s * 1000)));
  }

  // NOTE: destructors automatically clean up remaining objects
  return 0;
}
