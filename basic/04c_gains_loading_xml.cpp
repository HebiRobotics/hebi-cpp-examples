/**
 * Save and load gains using the cross-API gains XML format
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * October 2018
 */

#include <iostream>
#include "lookup.hpp"
#include "group_command.hpp"

using namespace hebi;

int main() {
  // Get group
  Lookup lookup;
  auto group = lookup.getGroupFromNames({"Test Family"}, {"Test Actuator" });

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  // Load the gains from a saved file
  GroupCommand cmd(group->size());
  // Set gains.  If this doesn't succeed, it may be because the number of
  // modules in the group doesn't match the number in the XML, or the file does
  // not exist or is corrupt.
  if (cmd.readGains("gains/example_gains.xml"))
  {
    std::cout << "Successfully read gains from file; now sending to module." << std::endl;
    group->sendCommand(cmd);
  }

  return 0;
}
