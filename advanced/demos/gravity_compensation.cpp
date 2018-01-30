/**
 * This file demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include <chrono>
#include <thread>

using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
using BracketType = hebi::robot_model::RobotModel::BracketType;
using LinkType = hebi::robot_model::RobotModel::LinkType;

Eigen::VectorXd getGravCompTorques(
  const hebi::robot_model::RobotModel& model,
  const Eigen::VectorXd& masses,
  const Eigen::VectorXd& angles,
  const Eigen::Vector3d& gravity)
{
  // Normalize gravity vector (to 1g, or 9.8 m/s^2)
  Eigen::Vector3d normed_gravity = gravity;
  normed_gravity /= normed_gravity.norm();
  normed_gravity *= 9.81;

  int num_dof = model.getDoFCount();
  int num_frames = model.getFrameCount(HebiFrameTypeCenterOfMass);

  hebi::robot_model::MatrixXdVector jacobians;
  model.getJ(HebiFrameTypeCenterOfMass, angles, jacobians);

  // Get torque for each module
  // comp_torque = J' * wrench_vector
  // (for each frame, sum this quantity)
  Eigen::VectorXd comp_torque(num_dof);
  comp_torque.setZero();

  // Wrench vector
  Eigen::VectorXd wrench_vec(6); // For a single frame; this is (Fx/y/z, tau x/y/z)
  wrench_vec.setZero();
  for (int i = 0; i < num_frames; ++i)
  {
    // Set translational part
    for (int j = 0; j < 3; ++j)
    {
      wrench_vec[j] = -normed_gravity[j] * masses[i];
    }

    // Add the torques for each joint to support the mass at this frame
    comp_torque += jacobians[i].transpose() * wrench_vec;
  }

  return comp_torque;
}

int main(int argc, char* argv[])
{
  // Note: this demo is written for a simple 3DOF arm with the kinematics
  // given below.  You can adapt to other systems by creating the correct
  // robot_model object and mass vector.

  // Try and get the requested groups.
  std::shared_ptr<hebi::Group> arm;
  {
    hebi::Lookup lookup;
    // Note -- can expand demo by adding in more modules to each group here.
    arm = lookup.getGroupFromNames({"HEBI"}, {"base", "shoulder", "elbow"});
    if (!arm)
    {
      std::cout << "Could not find arm group - check names!" << std::endl;
      return -1;
    }
  }

  // Create a simple kinematic description of the arm 
  hebi::robot_model::RobotModel model;
  model.addActuator(ActuatorType::X5_4);
  model.addBracket(BracketType::X5HeavyLeftInside);
  model.addActuator(ActuatorType::X5_4);
  model.addLink(LinkType::X5, 0.18, M_PI);
  model.addActuator(ActuatorType::X5_4);
  model.addLink(LinkType::X5, 0.28, 0);

  Eigen::VectorXd masses(model.getFrameCount(HebiFrameTypeCenterOfMass));
  model.getMasses(masses);

  arm->setCommandLifetimeMs(100);

  hebi::GroupCommand cmd(arm->size());

  Eigen::Vector3d gravity;
  gravity << 0, 0, -1;
 
  // Add a feedback handler to send feedback from one module to control the
  // other
  arm->addFeedbackHandler(
    [arm, &model, &masses, &gravity, &cmd](const hebi::GroupFeedback& feedback)->void
      {
        cmd.setEffort(getGravCompTorques(model, masses, feedback.getPosition(), gravity));
        arm->sendCommand(cmd);
      });

  // Start feedback callbacks
  arm->setFeedbackFrequencyHz(100);

  std::this_thread::sleep_for(std::chrono::seconds(20));

  // Stop the async callback before returning and deleting objects.
  arm->setFeedbackFrequencyHz(0);
  arm->clearFeedbackHandlers();

  // NOTE: destructors automatically clean up remaining objects
  return 0;
}
