/**
 * Generate a trajectory and execute it.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * September 2018
 */

#include "trajectory.hpp"
#include "Eigen/Eigen"

#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "command.hpp"
#include <math.h>
#include <chrono>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

int main()
{
  // Get group
  hebi::Lookup lookup;
  auto group = lookup.getGroupFromNames({"Test Family"}, {"Test Actuator" });

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  // Get the current feedback from the module; this allows us to start at the
  // current position
  int num_joints = group->size();
  hebi::GroupFeedback fbk(num_joints);
  
  if (!group->getNextFeedback(fbk)) {
    printf("Error getting feedback.\n");
    return -1;
  }

  // Create a smooth trajectory starting at the current position, moving to half
  // a rotation away, and coming back to the starting position.
  // Each column is a separate waypoint; each row is a different joint.
  // You can create position, velocity, and acceleration waypoints; the velocity
  // and acceleration waypoints default to zero at the endpoints and
  // unconstrained in the interior points.
  Eigen::MatrixXd positions(num_joints,3);
  auto offset = Eigen::VectorXd::Constant(num_joints, M_PI);
  auto current_pos = fbk.getPosition();

  positions.col(0) = current_pos;
  positions.col(1) = current_pos + offset;
  positions.col(2) = current_pos;

  // The times to reach each waypoint (in seconds)
  Eigen::VectorXd time(3);
  time << 0, 3, 6;

  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions);

  // Start logging in the background
  group->startLog("logs");

  // Follow the trajectory
  hebi::GroupCommand cmd(num_joints);
  double duration = trajectory->getDuration();
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);

  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);

  while (t.count() < duration) {
    // "getNextFeedback" serves to rate limit the loop without calling sleep
    group->getNextFeedback(fbk);
    t = std::chrono::system_clock::now() - start;

    // Pass "nullptr" in to ignore a term.  Position and time are always returned.
    trajectory->getState(t.count(), &pos_cmd, &vel_cmd, nullptr);
    cmd.setPosition(pos_cmd);
    cmd.setVelocity(vel_cmd);
    group->sendCommand(cmd);
  }

  // Stop logging
  auto log_file = group->stopLog();

  return 0;
}
