/**
 * This file demonstrates the group lookup capabilities of the API. The lookup
 * allows you to create groups of modules based on which ones are available on
 * the network.
 */

#include "lookup.hpp"
#include "mac_address.hpp"
#include <chrono>
#include <iostream>
#include <thread>

/**
 * Simple helper function to print out if the group is valid, based on number of
 * modules (-1 for invalid group).
 */
void checkGroup(int num_modules)
{
  if (num_modules > 0)
    std::cout << "Found group with " << num_modules << " modules." << std::endl;
  else
    std::cout << "Group not found on network." << std::endl;
}

/**
 * Attempts to look for several groups on the network (hardcoded addresses below),
 * and retrieve references to them.
 */
int main()
{
  // Create the lookup object
  hebi::Lookup lookup;

  // Wait for the module list to populate, and print out its contents
  std::this_thread::sleep_for(std::chrono::seconds(2));
  {
    std::cout << "Modules found on network (Family|Name):" << std::endl;
    std::shared_ptr<hebi::Lookup::EntryList> entry_list = lookup.getEntryList();
    for (auto entry : *entry_list)
    {
      std::cout << entry.family_ << " | " << entry.name_ << std::endl;
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  // Define module names/addresses:
  std::vector<hebi::MacAddress> macs;
  macs.emplace_back(hebi::MacAddress::fromBytes(0x00, 0x1e, 0xc0, 0x8d, 0xe0, 0x5c));
  macs.emplace_back(hebi::MacAddress::fromBytes(0x00, 0x1e, 0xc0, 0x8d, 0x79, 0xd8));

  std::vector<std::string> families;
  families.push_back("CommsTest");

  std::vector<std::string> names;
  names.push_back("One");
  names.push_back("Two");

  // Test out lookup methods; use a 4 second timeout.
  long timeout_ms = 4000;

  std::cout << "Looking up group by MAC addresses." << std::endl;
  std::shared_ptr<hebi::Group> group = lookup.getGroupFromMacs(macs, timeout_ms);
  checkGroup(group ? group->size() : -1);

  std::cout << "Looking up group by name." << std::endl;
  group = lookup.getGroupFromNames(families, names, timeout_ms);
  checkGroup(group ? group->size() : -1);

  std::cout << "Looking up group by family." << std::endl;
  group = lookup.getGroupFromFamily(families[0], timeout_ms);
  checkGroup(group ? group->size() : -1);

  std::cout << "Looking up connected group by mac." << std::endl;
  group = lookup.getConnectedGroupFromMac(macs[0], timeout_ms);
  checkGroup(group ? group->size() : -1);

  std::cout << "Looking up connected group by name." << std::endl;
  group = lookup.getConnectedGroupFromName(families[0], names[0], timeout_ms);
  checkGroup(group ? group->size() : -1);

  // NOTE: destructors automatically clean up objects
  return 0;
}
