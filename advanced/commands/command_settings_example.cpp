/**
 * This file demonstrates the ability to command settings to a group.
 */

#include <sstream>

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

  for (int module_index = 0; module_index < num_modules; module_index++)
  {
    std::ostringstream name;
    name << module_index;
    // Set the name for this module, based on its index within the group.
    // (Note -- we do not save this value, so upon module reset this will be
    // cleared)
    command[module_index].settings().name().set(name.str());
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

  // NOTE: destructors automatically clean up remaining objects
  return 0;
}
