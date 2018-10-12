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
#include "group_info.hpp"

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

  // Get the gains that are currently active on the actuator and save them
  GroupInfo info(group->size());
  if (!group->requestInfo(info)) {
    std::cout << "Did not receive gains from the module." << std::endl;
    return -1;
  }

  // Save gains to a file. If this doesn't succeed, it probably indicates the
  // directory doesn't exist.
  if (info.writeGains("gains/my_actuator_gains.xml"))
    std::cout << "Successfully read gains from module and wrote to file." << std::endl;

  return 0;
}
