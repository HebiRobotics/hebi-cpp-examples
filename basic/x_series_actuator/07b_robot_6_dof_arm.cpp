/**
 * Put everything together to control a 6-DoF arm.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * October 2018
 */

#include <iostream>
#include <chrono>
#include <thread>

#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"

// The examples provide this utility function for computing gravity compensation
// efforts.
#include "util/grav_comp.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "log_file.hpp"
#include "hebi_charts.hpp"

using namespace hebi;

/// A helper function to create a group from named modules, and set specified
/// gains on the modules in that group.
std::shared_ptr<Group> getGroup() {
  // Get group
  std::vector<std::string> families {"HEBI"};
  std::vector<std::string> names {"J1_base","J2_shoulder","J3_elbow","J4_wrist1","J5_wrist2","J6_wrist3"};
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames(families, names);
  if (!group)
    return nullptr;

  // Set gains
  GroupCommand gains_command(group->size());
  if (!gains_command.readGains("gains/6-DoF_arm_gains.xml"))
    return nullptr;
  if (!group->sendCommandWithAcknowledgement(gains_command))
    return nullptr;

  return group;
}

/// A helper function to actually execute the trajectory on a group of modules
void executeTrajectory(
    Group& group,
    const robot_model::RobotModel& model,
    const trajectory::Trajectory& trajectory,
    GroupFeedback& feedback) {

  // Set up command object, timing variables, and other necessary variables
  size_t num_joints = group.size();
  GroupCommand command(num_joints);
  double duration = trajectory.getDuration();
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);
  Eigen::VectorXd acc_cmd(num_joints); // note that the acceleration command is read from
                                       // the trajectory; you need dynamics info before
                                       // converting to efforts to send to the robot
  Eigen::VectorXd eff_cmd(num_joints);
  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  Eigen::VectorXd masses;
  model.getMasses(masses);

  while (t.count() < duration) {
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
    t = std::chrono::system_clock::now() - start;

    // Get new commands from the trajectory
    trajectory.getState(t.count(), &pos_cmd, &vel_cmd, &acc_cmd);

    // Calculate commanded efforts to assist with tracking the trajectory.
    // Gravity Compensation uses knowledge of the arm's kinematics and mass to
    // compensate for the weight of the arm.  Dynamic Compensation uses the
    // kinematics and mass to compensate for the commanded accelerations of
    // the arm.
    eff_cmd = hebi::util::getGravityCompensationEfforts(model, masses, feedback);
    // NOTE: dynamic compensation effort computation has not yet been added to
    // the C++ API utility functions.  These are coming soon! TODO
    // eff_cmd += hebi::util::DynamicCompensation::getEfforts(...);

    // Fill in the command and send commands to the arm    
    command.setPosition(pos_cmd);
    command.setVelocity(vel_cmd);
    command.setEffort(eff_cmd);
    group.sendCommand(command);
  }
}

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
/// The main function which actually executes to run the example
int run(int, char**) {
  // Get group of modules and set gains.
  std::shared_ptr<Group> group = getGroup();
  if (!group) {
    std::cout
      << "Group not found, or could not send gains to the modules. Check that the" << std::endl
      << "correct modules on the network, the connection is robust, and that the" << std::endl
      << "gains XML file is in the correct relative path." << std::endl;
    return -1;
  }

  // Load robot model/kinematics and gains
  auto model = robot_model::RobotModel::loadHRDF("hrdf/6-DoF_arm_example.hrdf");
  if (!model)
  {
    std::cout << "Could not load HRDF!" << std::endl;
    return -1;
  }
 
  // Go to the XYZ positions at four corners of the box, and create a rotation
  // matrix that has the end effector point straight forward.
  Eigen::MatrixXd xyz_targets(3, 4);
  xyz_targets << 0.40,  0.40,  0.40,  0.40, // x [m]
                 0.20,  0.20, -0.20, -0.20, // y [m]
                 0.10,  0.50,  0.50,  0.10; // z [m]
  Eigen::Matrix3d rotation_target =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

  // Convert these to joint angle waypoints using IK solutions for each of the
  // xyz locations, as well as the desired orientation of the end effector.
  // Copy the initial waypoint at the end so we close the square

  // Choose an "elbow up" initial configuration for IK
  Eigen::VectorXd elbow_up_angles(6);
  elbow_up_angles << 0, M_PI/4.0, M_PI/2.0, M_PI/4.0, -M_PI, M_PI/2.0; // [rad]

  Eigen::MatrixXd joint_targets(group->size(), xyz_targets.cols() + 1);
  Eigen::VectorXd ik_res_angles;
  for (int col = 0; col < xyz_targets.cols(); ++col) {
    model->solveIK(
      elbow_up_angles, // Initial joint angles
      ik_res_angles, // IK result
      robot_model::EndEffectorSO3Objective(rotation_target), // Objective
      robot_model::EndEffectorPositionObjective(xyz_targets.col(col))); // Objective
    joint_targets.col(col) = ik_res_angles;
  }
  joint_targets.col(joint_targets.cols() - 1) = joint_targets.col(0);

  // Set up feedback object, and start logging 
  GroupFeedback feedback(group->size());
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  // Get a trajectory from the current position to the first corner of the box: 
  Eigen::MatrixXd waypoints(group->size(), 2);
  group->getNextFeedback(feedback);
  waypoints.col(0) = feedback.getPosition();
  waypoints.col(1) = joint_targets.col(0);
  Eigen::VectorXd time(2);
  time << 0, 5; // Seconds for the motion; we do this slowly
  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, waypoints);

  // Call a helper function (above) to execute this motion on the robot
  executeTrajectory(*group, *model, *trajectory, feedback);

  // Go to all 4 corners.  Calculate new point-to-point trajectories one at a
  // time.
  time(1) = 3; // seconds for the move; we do this a little bit more quickly
  for (int col = 0; col < joint_targets.cols() - 1; ++col) {
    waypoints.col(0) = joint_targets.col(col);
    waypoints.col(1) = joint_targets.col(col + 1);
    trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, waypoints);
    executeTrajectory(*group, *model, *trajectory, feedback);
  }

  // Stop logging
  auto log_file = group->stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  //plot logged position, velocity and efforts for each module
  std::vector<std::vector<double>> pos;
  std::vector<std::vector<double>> vel;
  std::vector<std::vector<double>> eff;
  pos.resize(group->size());
  vel.resize(group->size());
  eff.resize(group->size());
  double t0{};
  std::vector<double> times;
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    for(size_t i = 0; i < group->size(); i++){
      pos[i].push_back(fbk.getPosition()[i]);
      vel[i].push_back(fbk.getVelocity()[i]);
      eff[i].push_back(fbk.getEffort()[i]);
    }
    if (t0 == 0)
      t0 = fbk.getTime();
    times.push_back(fbk.getTime() - t0);
  }
  
  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::Window window;
    window.setGridSize(1, 3);
    auto pos_chart = window.addChart(0, 0, 1, 1);
    pos_chart.setTitle("Position");
    for(size_t i = 0; i < group->size(); i++){
      auto title = (std::string("module ") + std::to_string(i));
      pos_chart.addLine(title, times, pos[i]);
    }
    auto vel_chart = window.addChart(0, 1, 1, 1);
    vel_chart.setTitle("Velocity");
    for(size_t i = 0; i < group->size(); i++){
      auto title = (std::string("module ") + std::to_string(i));
      vel_chart.addLine(title, times, vel[i]);
    }
    auto eff_chart = window.addChart(0, 1, 1, 1);
    eff_chart.setTitle("Effort");
    for(size_t i = 0; i < group->size(); i++){
      auto title = (std::string("module ") + std::to_string(i));
      eff_chart.addLine(title, times, eff[i]);
    }
    window.show();

    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}
