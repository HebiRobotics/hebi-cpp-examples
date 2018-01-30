/**
 * This file demonstrates the ability to get feedback from a group.
 */

#include "lookup.hpp"
#include "group_feedback.hpp"
#include "mac_address.hpp"
#include <chrono>
#include <cmath>
#include <thread>

// This is a callback function for the async feedback loop:
void feedback_callback(const hebi::GroupFeedback& feedback)
{
  // Print out position of first module:
  const auto& position = feedback[0].actuator().position();
  if (position.has())
    std::cout << position.get() << std::endl;
  else
    std::cout << "no position feedback!" << std::endl;
}

// This is a callback function for the async feedback loop using an external parameter
void feedback_callback_with_param(const hebi::GroupFeedback& feedback, std::shared_ptr<int> feedback_count)
{
  // Print out position of first module:
  const auto& position = feedback[0].actuator().position();
  if (position.has())
    std::cout << position.get() << std::endl;
  else
    std::cout << "no position feedback!" << std::endl;

  // Increment feedback count:
  (*feedback_count)++;
}

int main(int argc, char* argv[])
{
  // Try and get the requested group.
  std::shared_ptr<hebi::Group> group;
  {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames({"X5-4"}, {"X5-0000", "X5-0001"});
    if (!group)
    {
      std::cout << "No group found!" << std::endl;
      return -1;
    }
  }

  // Three examples of using C++11 functions to register feedback handlers and to
  // share data.
  // (Because these occur in a separate thread, use standard multi-threaded
  // programming practice to control access to the variables whilst avoiding
  // deadlocks)

  // See http://en.cppreference.com/w/cpp/utility/functional/function
  // Note: this could also wrap member functions via std::bind
  
  // If you want to use a variable past the scope of this calling function,
  // use a shared pointer to keep track of all references to this integer.
  // Allocating on the stack will result in UB if accessed after 'main()' returns.
  std::shared_ptr<int> total_feedback_calls = std::make_shared<int>(0);
  // This calls an external (free) function:
  std::function<void(const hebi::GroupFeedback&)> free_function = feedback_callback;
  group->addFeedbackHandler(free_function);
  // This calls an free function with an extra parameter
  group->addFeedbackHandler([total_feedback_calls](const hebi::GroupFeedback& fbk)->void
    { feedback_callback_with_param(fbk, total_feedback_calls); });
  // This calls a lambda function:
  group->addFeedbackHandler(
    [total_feedback_calls](const hebi::GroupFeedback& feedback)->void
      {
        // Print out position of first module:
        const auto& position = feedback[0].actuator().position();
        if (position.has())
          std::cout << position.get() << std::endl;
        else
          std::cout << "no position feedback!" << std::endl;

        // Increment feedback count:
        ++*total_feedback_calls;
      });

  // Start 200Hz feedback loop.
  std::cout << "Starting asynchronous feedback callbacks" << std::endl;
  group->setFeedbackFrequencyHz(200);
  // Wait 10 seconds. This should result in about 2,000 callbacks (if you have
  // all handlers enabled above, this will result in about 4,000)
  int wait_period_s = 10;
  std::this_thread::sleep_for(std::chrono::seconds(wait_period_s));
  // Stop the feedback loop, and unrelease our callback:
  group->setFeedbackFrequencyHz(0);
  group->clearFeedbackHandlers();

  std::cout << "Feedback callback thread stopped; " << *total_feedback_calls << " callbacks were made." << std::endl;

  // NOTE: destructors automatically clean up remaining objects
  return 0;
}
