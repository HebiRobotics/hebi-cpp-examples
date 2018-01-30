/**
 * An example to lookup a group based on command line arguments.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 23 Sep 2015
 */ 

#include "lookup_helpers.cpp"

int main(int argc, char* argv[])
{
  // Try and get the requested group
  std::shared_ptr<hebi::Group> group = getGroupFromArgs(argc, argv);
  if (!group)
  {
    std::cout << "Group not found on network." << std::endl << std::endl;
    return -1;
  }

  std::cout << "Found group with " << group->size() << " modules." << std::endl << std::endl;

  return 0;
}
