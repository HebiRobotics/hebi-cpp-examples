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

  // Add a callback to react to feedback received on a background thread
  // Note: We use a C++11 "lambda function" here to pass in a function pointer,
  // but you can also pass in a C-style function pointer with the signature:
  //   void func(const hebi::GroupFeedback& group_fbk);
  group->addFeedbackHandler([](const GroupFeedback& group_fbk)
  {
    std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
  });

  // Wait for 10 seconds, and then stop.
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  group->clearFeedbackHandlers();

  return 0;
}
