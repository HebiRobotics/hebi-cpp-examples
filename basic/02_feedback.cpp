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
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "family" }, { "base", "shoulder", "elbow" });
  // This is by default "100"; setting this to zero here demonstrates the
  // ability to send single feedback requests.
  group->setFeedbackFrequencyHz(0);

  if (!group)
  {
    std::cout << "Group not found!";
    return -1;
  }

  // Retrieve feedback with a syncronous blocking call.
  GroupFeedback group_fbk(group->size());
  if (!group->sendFeedbackRequest())
  {
    std::cout << "Could not send feedback request." << std::endl;
    return -1;
  }

  if (group->getNextFeedback(group_fbk))
  {
    std::cout << "Got feedback.  Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
    std::cout << "Can also check individual module position values:" << std::endl;
    for (size_t i = 0; i < group_fbk.size(); ++i)
    {
      auto& pos_fbk = group_fbk[i].actuator().position();
      if (pos_fbk)
        std::cout << pos_fbk.get() << std::endl;
      else
        std::cout << "No feedback" << std::endl;
    }
  }

  // Add a background thread to request module feedback, and a callback to react to feedback.
  group->addFeedbackHandler([](const GroupFeedback& group_fbk)
  {
    std::cout << "Got feedback." << std::endl;
  });

  // Start responding to feedback at 25 Hz; wait 1 second, and then stop.
  group->setFeedbackFrequencyHz(25);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  group->clearFeedbackHandlers();

  return 0;
}
