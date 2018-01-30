#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"

using namespace hebi;

int main()
{
  // Create a Lookup object
  Lookup lookup;

  // Wait 2 seconds for the module list to populate, and print out its contents
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  std::shared_ptr<Lookup::EntryList> entry_list = lookup.getEntryList();
  for (auto entry : *entry_list)
  {
    std::cout << "Name: " << entry.name_ << std::endl << "Family: " << entry.family_ << std::endl << std::endl;
  }
  std::cout << std::endl;

  // Actually create the group
  std::shared_ptr<Group> group = lookup.getGroupFromNames({"family"}, {"base", "shoulder", "elbow"});

  if (!group)
  {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  std::cout << "Found group on network: size " << group->size() << std::endl;
  return 0;
}
