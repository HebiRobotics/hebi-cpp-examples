#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"

using namespace hebi;

int main()
{
  // Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "family" }, { "base", "shoulder", "elbow" });

  if (!group)
  {
    std::cout << "Group not found!";
    return -1;
  }
  
  // Add a callback function to respond to feedback with a "virtual spring" command
  double spring_constant = -10; // Nm / rad
  GroupCommand group_command(group->size());
  group->addFeedbackHandler([&group, &group_command, &spring_constant](const GroupFeedback& group_fbk)
  {
    // Apply Hooke's law: F = -k * x
    group_command.setEffort(spring_constant * group_fbk.getPosition());
    group->sendCommand(group_command);
  });
  
  // Control the robot at 100 Hz for 30 seconds
  std::this_thread::sleep_for(std::chrono::seconds(30));
  group->clearFeedbackHandlers();

  return 0;
}
