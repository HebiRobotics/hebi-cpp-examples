/**
 * This file demonstrates the ability to set the spring constants.
 */

#include <stdio.h>
#include <math.h>

#include "lookup.hpp"
#include "group_command.hpp"
#include "command.hpp"
#include "mac_address.hpp"
#include "lookup_helpers.cpp"

int main(int argc, char* argv[])
{
  // Try and get the requested group.
  std::unique_ptr<hebi::Group> group = getGroupFromArgs(argc, argv);
  if (!group)
  {
    std::cout << "No group found!" << std::endl;
    return -1;
  }

  int num_modules = group->size();

  // Create a command object; this can be sent to the group
  hebi::GroupCommand command(num_modules);

  long timeout_ms = 1000;
  hebi::GroupInfo info_orig(num_modules);

  std::cout << "Getting current spring constants." << std::endl;
  if (group->requestInfo(info_orig, timeout_ms))
  {
    // Print out the spring constants
    std::cout << "Spring constants:";
    for (int module_index = 0; module_index < num_modules; module_index++)
    {
      auto& spring_constant = info_orig[module_index].settings().actuator().springConstant();
      if (spring_constant.has())
        std::cout << " " << spring_constant.get();
      else
        std::cout << " (no data)";
    }
    std::cout << std::endl;
  }
  else
  {
    // NOTE: destructors automatically clean up remaining objects
    std::cout << "Get failed!" << std::endl << "Quitting early..." << std::endl;
    return -1;
  }

  // Set the spring constants:
  printf("Setting spring constants to 1.\n");
  float new_spring_constant = 1.0f;
  for (int module_index = 0; module_index < num_modules; module_index++)
  {
    // Set the spring constant for this module.
    // (Note -- we do not save this value, so upon module reset this will be
    // cleared)
    command[module_index].settings().actuator().springConstant().set(new_spring_constant);
  }

  if (group->sendCommandWithAcknowledgement(command, timeout_ms))
  {
    std::cout << "Got acknowledgement." << std::endl;
  }
  else
  {
    std::cout << "Did not receive acknowledgement!" << std::endl;
  }

  // Checking spring constant (use new info message so we don't lose old values)
  hebi::GroupInfo info_new(num_modules);
  std::cout << "Getting new spring constants:" << std::endl;
  if (group->requestInfo(info_new, timeout_ms))
  {
    // Print out the spring constants
    std::cout << "Spring constants:";
    for (int module_index = 0; module_index < num_modules; module_index++)
    {
      auto& spring_constant = info_orig[module_index].settings().actuator().springConstant();
      if (spring_constant.has())
        std::cout << " " << spring_constant.get();
      else
        std::cout << " (no data)";
    }
    std::cout << std::endl;
  }
  else
  {
    std::cout << "Get failed!" << std::endl;
  }

  // Reset the spring constants:
  std::cout << "Resetting spring constants to previous values." << std::endl;
  for (int module_index = 0; module_index < num_modules; module_index++)
  {
    // Set the spring constant for this module to the previous value, or nothing
    // if we didn't get it back before (this is reading from the previously
    // received info packet)
    auto& orig_spring_constant = info_orig[module_index].settings().actuator().springConstant();
    if (orig_spring_constant.has())
    {
      command[module_index].settings().actuator().springConstant().set(orig_spring_constant.get());
    }
    else
    {
      // No value originally found, so just don't send anything to this module
      command[module_index].settings().actuator().springConstant().clear();
    }
  }

  if (group->sendCommandWithAcknowledgement(command, timeout_ms))
  {
    std::cout << "Got acknowledgement." << std::endl;
  }
  else
  {
    std::cout << "Did not receive acknowledgement!" << std::endl <<
                 "Warning: spring constants may not be set back to previous values!" << std::endl;
  }

  // NOTE: destructors automatically clean up remaining objects
  return 0;
}
