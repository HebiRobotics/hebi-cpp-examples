/**
 * This file demonstrates the ability to command a group.
 */

#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>

#include "lookup.hpp"
#include "group_command.hpp"
#include "command.hpp"

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

  // Send commands to the group in a loop.
  // Note that these packets may be dropped if network traffic is too high, so
  // be sure to close a feedback loop at the high level!
  float period = 0.5f;
  for (float t = 0.0f; t < 10.0f; t += period)
  {
    // Set the actuator command position field for all the modules
    for (int module_index = 0; module_index < num_modules; module_index++)
    {
      // Set the position for this module, offsetting each with respect to other
      // modules.
      command[module_index].actuator().position().set(sin(t * 0.5f + module_index * 0.25f));
    }

    group->sendCommand(command);
    std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period * 1000)));
  }

  // For critical packets, we can verify that they were sent by requesting
  // confirmation from the group.  If the acknowledgement function returns
  // 'true', it got positive confirmation. If it returns 'false', EITHER:
  // - the sent command was dropped
  // - the sent command was received by the group, but its response was either
  //   dropped or the timeout period expired before receipt of the group.
  // Again, a high-level process should intelligently handle these conditions!
  // Note that this is a blocking call, and so for high-frequency applications,
  // SendCommand should be used instead.
  period = 0.5f;
  long timeout_ms = 100;
  for (float t = 0.0f; t < 10.0f; t += period)
  {
    // Set the actuator command position field for all the modules
    for (int module_index = 0; module_index < num_modules; module_index++)
    {
      // Set the position for this module, offsetting each with respect to other
      // modules.
      command[module_index].actuator().position().set(sin(t * 0.5f + module_index * 0.25f));
    }

    if (group->sendCommandWithAcknowledgement(command, timeout_ms))
    {
      std::cout << "Got acknowledgement." << std::endl;
    }
    else
    {
      std::cout << "Did not receive acknowledgement!" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period * 1000)));
  }

  // NOTE: destructors automatically clean up group command and group
  return 0;
}
