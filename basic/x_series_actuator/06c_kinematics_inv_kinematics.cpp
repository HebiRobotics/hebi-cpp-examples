#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "util/plot_functions.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace hebi;
using ActuatorType = robot_model::ActuatorType;
using BracketType = robot_model::BracketType;
using LinkType = robot_model::LinkType;

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
  auto model =
    robot_model::RobotModel::loadHRDF("hrdf/3-DoF_arm_example.hrdf");
  if (!model)
  {
    std::cout << "Could not load HRDF!" << std::endl;
    return -1;
  }

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
  model->solveIK(
    initial_joint_angles,
    ik_result_joint_angles,
    robot_model::EndEffectorPositionObjective(target_xyz));

  std::cout << std::endl << "Target position: " << std::endl << target_xyz.transpose() << std::endl;
  std::cout << "IK joint angles: " << std::endl << ik_result_joint_angles.transpose() << std::endl;
  Eigen::Matrix4d transform;
  model->getEndEffector(ik_result_joint_angles, transform);
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
  model->solveIK(
    initial_joint_angles,
    ik_result_joint_angles,
    robot_model::EndEffectorPositionObjective(target_xyz),
    robot_model::JointLimitConstraint(min_positions, max_positions)
  );

  std::cout << "Target position: " << std::endl << target_xyz.transpose() << std::endl;
  std::cout << "IK joint angles: " << std::endl << ik_result_joint_angles.transpose() << std::endl;
  hebi::robot_model::Matrix4dVector transforms;
  model->getFK(robot_model::FrameType::Output, ik_result_joint_angles, transforms);

  // plot frames on a 3d graph
  transforms.emplace(transforms.begin(),Eigen::Matrix<double,4,4>::Identity());
  std::vector<std::vector<double>> lines_x;
  std::vector<std::vector<double>> lines_y;
  std::vector<std::vector<double>> lines_z;

  for(size_t j = 0; j < transforms.size(); ++j) {
    plot_3dtriad(transforms[j],&lines_x,&lines_y,&lines_z, static_cast<bool>(j));
  }
  plt::pause(1);

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
