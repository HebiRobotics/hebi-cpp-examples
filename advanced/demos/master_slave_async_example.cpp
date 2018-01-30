/**
 * This file demonstrates master-slave control from one module to another, with
 * the feedback loop handled by the API. There must be two modules in the group;
 * the first one controls the second.
 */

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
  // Try and get the requested groups.
  std::shared_ptr<hebi::Group> master;
  std::shared_ptr<hebi::Group> slave;
  {
    hebi::Lookup lookup;
    // Note -- can expand demo by adding in more modules to each group here.
    master = lookup.getGroupFromNames({"HEBI"}, {"master"});
    slave = lookup.getGroupFromNames({"HEBI"}, {"slave"});
    if (!master || !slave)
    {
      std::cout << "One of the groups not found!" << std::endl;
      return -1;
    }
    if (master->size() != slave->size())
    {
      std::cout << "Groups must be same size for master/slave control." << std::endl;
      return -1;
    }
  }

  hebi::GroupCommand cmd(slave->size());

  // Add a feedback handler to send feedback from one module to control the
  // other
  master->addFeedbackHandler(
    [&master, &slave, &cmd](const hebi::GroupFeedback& feedback)->void
      {
        cmd.setPosition(feedback.getPosition());
        slave->sendCommand(cmd);
      });

  // Start feedback callbacks
  master->setFeedbackFrequencyHz(200);

  std::this_thread::sleep_for(std::chrono::seconds(20));

  // Stop the async callback before returning and deleting objects.
  master->setFeedbackFrequencyHz(0);
  master->clearFeedbackHandlers();

  // NOTE: destructors automatically clean up remaining objects
  return 0;
}
