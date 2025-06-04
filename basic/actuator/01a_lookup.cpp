/**
 * Initialize the lookup of modules and display information for any modules
 * on the network. A 'module' can be an actuator, and I/O board, or a 
 * mobile device running the Mobile I/O app.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * HEBI Robotics
 * September 2018
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "lookup.hpp"

using namespace hebi;

int main() {
  // Create a Lookup object
  Lookup lookup;

  // Wait 2 seconds for the module list to populate, and then print out its contents
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  std::cout << std::endl;
  auto entry_list = lookup.getEntryList();
  for (auto entry : *entry_list)
    std::cout
      << "Name: " << entry.name_ << std::endl
      << "Family: " << entry.family_ << std::endl << std::endl;

  std::cout << std::endl
    << " NOTE: " << std::endl
    << "  The listing above should show the information for all the modules" << std::endl
    << "  on the local network.  If this is empty make sure that the modules" << std::endl
    << "  are connected, powered on, and that the status LEDs are displaying" << std::endl
    << "  a green soft-fade." << std::endl;

  return 0;
}
