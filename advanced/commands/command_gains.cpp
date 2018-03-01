#include "trajectory.hpp"
#include "Eigen/Eigen"

#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "command.hpp"
#include <cmath> // for nan
#include <math.h>
#include <chrono>
#include <thread>

int main()
{
  // Get a group
  std::shared_ptr<hebi::Group> group;

  {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames({ "family" }, { "base", "shoulder", "elbow" });
  }

  if (!group)
  {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  hebi::GroupCommand cmd(group->size());

  // Set gains.  If this doesn't succeed, it may be because the number of
  // modules in the group doesn't match the number in the XML, or the file was
  // corrupt.
  if (cmd.readGains("gains.xml"))
  {
    std::cout << "Successfully read gains from file; now sending to module." << std::endl;
    group->sendCommand(cmd);
  }

  return 0;
}
