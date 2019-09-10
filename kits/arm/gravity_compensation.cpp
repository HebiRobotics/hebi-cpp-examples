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
#include "lookup.hpp"
#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
  // Note: this demo is written for a simple 3DOF arm with the kinematics
  // given below.  You can adapt to other systems by creating the correct
  // robot_model object and mass vector.
  // std::unique_ptr<hebi::ArmContainer> arm = hebi::ArmContainer::create3Dof();
/////////////////
  hebi::Lookup lookup;
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow", "Wrist1"};//, "Wrist 2", "Wrist 3"};


  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;

  // hebi::robot_model::RobotModel model;
  // std::unique_ptr<hebi::robot_model::RobotModel> model;
  std::unique_ptr<hebi::robot_model::RobotModel> model(
    new hebi::robot_model::RobotModel());

  model -> addActuator(ActuatorType::X8_9);
  model -> addBracket(BracketType::X5HeavyRightInside);
  model -> addActuator(ActuatorType::X8_16);
  model -> addLink(LinkType::X5, 0.3, M_PI);
  model -> addActuator(ActuatorType::X8_9);
  model -> addLink(LinkType::X5, 0.3, 0);
  model -> addActuator(ActuatorType::X5_9);
  model -> addBracket(BracketType::X5LightRight);
  // model -> addActuator(ActuatorType::X5_4);
  // model -> addBracket(BracketType::X5LightLeft);
  // model -> addActuator(ActuatorType::X5_4);


  // std::unique_ptr<hebi::robot_model::RobotModel> *robo = &model;

  std::shared_ptr<hebi::Group> group = lookup.getGroupFromNames(family, names);
  if (!group)
    {
      std::cout << "Could not find arm group - check names!" << std::endl;
    }

  std::unique_ptr<hebi::ArmContainer> arm(new hebi::ArmContainer(group, std::move(model)));
 ///////////
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
