#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**)
{
  constexpr double PI = 3.14159265358979323846;
  //////////////////////////////////////
  // Set up group and robot_model
  //////////////////////////////////////
  
  // Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "family" }, { "base", "shoulder", "elbow" });

  if (!group)
  {
    std::cout << "Group not found!\n";
    return -1;
  }

  // Create a simple kinematic description of the arm 
  auto model =
    robot_model::RobotModel::loadHRDF("hrdf/3-DoF_arm_example.hrdf");
  if (!model)
  {
    std::cout << "Could not load HRDF!\n";
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
    std::cout << "Couldn't get feedback!\n";
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
  min_positions << -PI, 0.25f, 0.25f;
  Eigen::VectorXd max_positions(group->size());
  max_positions << PI, 1.0f, 1.0f;

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
  transforms.emplace(transforms.begin(), Eigen::Matrix<double,4,4>::Identity());

  if (hebi::charts::framework::isLoaded()) {
    hebi::charts::Chart3d chart;
    chart.show();
    for(size_t j = 0; j < transforms.size(); ++j) {
      auto triad = chart.addTriad(0.075);
      Eigen::Quaterniond q(Eigen::Matrix3d(transforms[j].topLeftCorner(3, 3)));
      triad.setOrientation(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d xyz = transforms[j].topRightCorner(3, 1);
      triad.setTranslation(xyz.x(), xyz.y(), xyz.z());
    }
    
    hebi::charts::framework::waitUntilWindowsClosed();
  }

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
