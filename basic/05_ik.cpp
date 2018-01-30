#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

using namespace hebi;
using ActuatorType = robot_model::RobotModel::ActuatorType;
using BracketType = robot_model::RobotModel::BracketType;
using LinkType = robot_model::RobotModel::LinkType;

int main()
{
  //////////////////////////////////////
  // Set up group and robot_model
  //////////////////////////////////////
  
  // Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "family" }, { "base", "shoulder", "elbow" });

  if (!group)
  {
    std::cout << "Group not found!";
    return -1;
  }

  // Create a simple kinematic description of the arm 
  robot_model::RobotModel model;
  model.addActuator(ActuatorType::X5_4);
  model.addBracket(BracketType::X5LightRight);
  model.addActuator(ActuatorType::X5_4);
  model.addLink(LinkType::X5, 0.18, M_PI);
  model.addActuator(ActuatorType::X5_4);
  model.addLink(LinkType::X5, 0.28, 0);

  Eigen::Vector3d target_xyz;
  target_xyz << 0.4, 0.0, 0.2;
  Eigen::VectorXd initial_joint_angles(group->size());
  Eigen::VectorXd ik_result_joint_angles(group->size());

  //////////////////////////////////////
  // Get position feedback from robot
  // to use as initial conditions for
  // local optimization.
  //////////////////////////////////////
  
  // Get feedback
  GroupFeedback group_fbk(group->size());
  
  if (!group->getNextFeedback(group_fbk))
  {
    std::cout << "Couldn't get feedback!";
    return -1;
  }

  for (size_t i = 0; i < group_fbk.size(); ++i)
  {
    // Note -- should check whether this is valid.
    initial_joint_angles(i) = group_fbk[i].actuator().position().get();
  }

  //////////////////////////////////////
  // Get IK solution with one objective
  //////////////////////////////////////

  // Just one objective:
  // Note - this is a numerical optimization, and can be significantly affecting
  // by initial conditions (e.g., seed joint angles)
  model.solveIK(
    initial_joint_angles,
    ik_result_joint_angles,
    robot_model::EndEffectorPositionObjective(target_xyz));

  std::cout << std::endl << "Target position: " << std::endl << target_xyz.transpose() << std::endl;
  std::cout << "IK joint angles: " << std::endl << ik_result_joint_angles.transpose() << std::endl;
  Eigen::Matrix4d transform;
  model.getEndEffector(ik_result_joint_angles, transform);
  std::cout << "FK of IK joint angles: " << std::endl << transform.topRightCorner<3,1>().transpose() << std::endl << std::endl;

  // Set joint limits to force a particular solution (elbow up, in this case)
  Eigen::VectorXd min_positions(group->size());
  min_positions << -M_PI, 0.25f, 0.25f;
  Eigen::VectorXd max_positions(group->size());
  max_positions << M_PI, 1.0f, 1.0f;

  //////////////////////////////////////
  // Get IK solution with multiple
  // objectives
  //////////////////////////////////////

  // Multiple objectives (note -- can add as many additional arguments as
  // desired).
  model.solveIK(
    initial_joint_angles,
    ik_result_joint_angles,
    robot_model::EndEffectorPositionObjective(target_xyz),
    robot_model::JointLimitConstraint(min_positions, max_positions)
  );

  std::cout << "Target position: " << std::endl << target_xyz.transpose() << std::endl;
  std::cout << "IK joint angles: " << std::endl << ik_result_joint_angles.transpose() << std::endl;
  model.getEndEffector(ik_result_joint_angles, transform);
  std::cout << "FK of IK joint angles: " << std::endl << transform.topRightCorner<3,1>().transpose() << std::endl << std::endl;

  //////////////////////////////////////
  // Send commands to the physical robot
  //////////////////////////////////////

  // Move the arm (note -- could use the Hebi Trajectory API to do this smoothly)
  GroupCommand group_cmd(group->size());
  group_cmd.setPosition(ik_result_joint_angles);

  // Note -- the arm will go limp after the 100 ms command lifetime, so we repeat
  // the command in a loop here until we terminate after approximately 5 seconds.
  for (int i = 0; i < 100; ++i)
  {
    group->sendCommand(group_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}
