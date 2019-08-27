/*
 * Make a group consisting of multiple modules.  A 'module' can be an actuator,
 * an I/O board, or a mobile device running the Mobile I/O app.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * HEBI Robotics
 * August 2019
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include"lookup.hpp"

using namespace hebi;

int main() {
  // Create a Lookup Object
  Lookup lookup;

  // Use Scope to select a module and change the name and family to 
  // match the names below. 
  std::string family_name("HEBI");
  std::vector<std::string> module_names({"IO Board", "Mobile IO", "Actuator 1"});

  // Actually create the group
  // (The "{}" are C++11 initializer braces that convert from a string to a
  // vector of strings)
  auto group = lookup.getGroupFromNames({family_name}, module_names);

  if (!group) {
    std::cout << std::endl
      << "Group not found! Check that the family and names of modules on the network" << std::endl
      << "match those given in the source file." << std::endl;
    return -1;
  }
  
  std::cout << std::endl << "Found group on network with " << group->size() << " modules." << std::endl;
  return 0;
}

