/**
 * This file demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "util/grav_comp.hpp"
#include "arm_container.hpp"
#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
  // Note: this demo is written for a simple 3DOF arm with the kinematics
  // given below.  You can adapt to other systems by creating the correct
  // robot_model object and mass vector.
  std::unique_ptr<hebi::ArmContainer> arm = hebi::ArmContainer::create3Dof();
  if (!arm)
    return -1;

  hebi::GroupCommand cmd(arm->getGroup().size());
  Eigen::Vector3d gravity(0, 0, -1);
 
  // Respond to every feedback packet with an effort command to cancel the
  // force due to gravity at this pose.
  arm->getGroup().addFeedbackHandler(
    [&arm, &gravity, &cmd](const hebi::GroupFeedback& feedback)->void
      {
        Eigen::VectorXd effort = hebi::util::GravityCompensation::getEfforts(
          arm->getRobotModel(),
          arm->getMasses(),
          feedback);
        cmd.setEffort(effort);
        arm->getGroup().sendCommand(cmd);
      });

  // Run for 60 seconds
  std::this_thread::sleep_for(std::chrono::seconds(60));
  arm->getGroup().clearFeedbackHandlers();

  return 0;
}
