#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"

using namespace hebi;

int main()
{
  // Get a module
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "family" }, { "name" });

  if (!group)
  {
    std::cout << "Group not found!";
    return -1;
  }
  
  // Set the command lifetime
  group->setCommandLifetimeMs(100);

  GroupCommand group_command(group->size());

  // Set the LED red
  //
  // The Color object arguments are RGBA, where the alpha channel is essentially
  // how much we "override" the module's control of the status LED.  Currently,
  // the value should either be "0" or "255" (in the future modules may support
  // more advanced color blending; currently, anything > 0 matches the behavior
  // of "255").  "0" indicates for the module to take control of the LED, and
  // "255" indicates the commanded color should override.
  group_command[0].led().set(Color(255, 0, 0, 255));
  group->sendCommand(group_command);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Set the LED Blue, and then clear the command.
  // Note that this "clear" does not return the LED to module control, but
  // rather remove any LED command from the command object, so when this is sent
  // to the module the LED state won't be affected.
  group_command[0].led().set(Color(0, 0, 255, 255));
  group_command[0].led().clear();
  group->sendCommand(group_command);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Set the LED to "module control" mode; the first three digits are ignored
  // if the 'alpha' value is zero.
  group_command[0].led().set(Color(0, 0, 0, 0));
  group->sendCommand(group_command);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Set the LED purple. Note that this override automatically sets the alpha
  // channel to "255" (e.g., arguments are RGB).
  group_command[0].led().set(Color(0, 255, 255));
  group->sendCommand(group_command);

  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  // Set the LED to module control.  Note that this default constructor
  // automatically sets the alpha channel to "0".
  group_command[0].led().set(Color());
  group->sendCommand(group_command);

  return 0;
}
