#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_feedback.hpp"

using namespace hebi;

int main()
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

  // Retrieve feedback with a blocking call to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run for
  // about 10 seconds here
  GroupFeedback group_fbk(group->size());
  for (size_t i = 0; i < 50; ++i)
  { 
    if (group->getNextFeedback(group_fbk))
    {
      std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;

      // Note -- can also retrieve individual module feedback; see API docs.
      // E.g., `group_fbk[0]` is the feedback from the first module.
    }
  }

  group->clearFeedbackHandlers();

  return 0;
}
