/**
 * This file demonstrates the ability to command settings to a group.
 */

#include <iostream>
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

  // Get a new name for the first module:
  std::string new_name = "";
  while (true)
  {
    std::cout << "Enter a new name for this module:" << std::endl;
    getline(std::cin, new_name);
 
    // NOTE: could do additional validation here.
    if (new_name.size() > 0)
      break;

    std::cout << std::endl << "Invalid name; please try again" << std::endl;
  }

  // Create a command object; this can be sent to the group
  hebi::GroupCommand command(num_modules);

  // Set a new name for the first module:
  command[0].settings().name().set(new_name);
  // Mark that settings should be persisted across reboots.
  command[0].settings().saveCurrentSettings().set();

  std::cout << "Setting and saving new module name. Restart and verify name has been reset." << std::endl;
  long timeout_ms = 100;
  if (group->sendCommandWithAcknowledgement(command, timeout_ms))
  {
    std::cout << "Got acknowledgement.\n" << std::endl;
  }
  else
  {
    std::cout << "Did not receive acknowledgement!\n" << std::endl;
  }

  // NOTE: destructors automatically clean up group command and group
  return 0;
}
