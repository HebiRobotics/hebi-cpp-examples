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
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

int main()
{
  // Get a group
  std::shared_ptr<hebi::Group> group;

  {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames({ "family" }, { "1", "2" });
  }

  if (!group)
  {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  // Get the current feedback from the module
  int num_joints = group->size();
  hebi::GroupFeedback fbk(num_joints);
  
  if (!group->getNextFeedback(fbk))
  {
    printf("Error getting feedback.\n");
    return -1;
  }

  // Create a smooth trajectory starting at the current position and ending at 0.
  // Position, velocity, and acceleration waypoints.  Each column is a separate
  // waypoint; each row is a different joint.
  Eigen::MatrixXd positions(num_joints,5);
  positions << fbk[0].actuator().position().get(), 0, M_PI_2, 0,         0,
               fbk[1].actuator().position().get(), 0, 0,        -M_PI_2, 0;

  // The times to reach each waypoint (in seconds)
  Eigen::VectorXd time(5);
  time << 0, 5, 10, 15, 20;

  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions);

  // Follow the trajectory
  hebi::GroupCommand cmd(num_joints);
  double period = 0.01;
  double duration = trajectory->getDuration();
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);
  for (double t = 0; t < duration; t += period)
  {
    // Pass "nullptr" in to ignore a term.  Position and time are always returned.
    trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
    cmd.setPosition(pos_cmd);
    cmd.setVelocity(vel_cmd);
    group->sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period * 1000)));
  }

  return 0;
}
