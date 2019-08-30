/**
 * Make a group consisting of a single module.  A 'module' can be an actuator,
 * an I/O board, or a mobile device running the Mobile I/O app.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * HEBI Robotics
 * September 2018
 */

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "lookup.hpp"

using namespace hebi;

int main() {
  // Create a Lookup object
  Lookup lookup;

  // Use Scope to select a module and change the name and family to
  // match the names below. Following examples will use the same names.
  std::string family_name("Test Family");
  std::string module_name("Test Actuator");

  // Actually create the group
  // (The "{}" are C++11 initializer braces that convert from a string to a
  // vector of strings)
  auto group = lookup.getGroupFromNames({family_name}, {module_name});

  if (!group) {
    std::cout << std::endl
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  std::cout << std::endl << "Created group from module " << family_name << " | " << module_name << "." << std::endl;
  return 0;
}
