/**
 * This file demonstrates the ability to command the control strategy.
 */

#include <stdio.h>
#include <math.h>

#include "lookup.hpp"
#include "group_command.hpp"
#include "command.hpp"
#include "mac_address.hpp"

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

  // Create a command object; this can be sent to the group
  hebi::GroupCommand command(num_modules);

  // Set the control strategy for all the modules
  hebi::Command::ControlStrategy new_strategy = hebi::Command::ControlStrategy::Strategy2;
  for (int module_index = 0; module_index < num_modules; module_index++)
  {
    command[module_index].settings().actuator().controlStrategy().set(new_strategy);
  }

  long timeout_ms = 100;
  if (group->sendCommandWithAcknowledgement(command, timeout_ms))
  {
    std::cout << "Got acknowledgement." << std::endl;
  }
  else
  {
    std::cout << "Did not receive acknowledgement!" << std::endl;
  }

  // NOTE: destructors automatically clean up group command and group
  return 0;
}
