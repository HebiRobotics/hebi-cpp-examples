#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
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

  // Add a callback function to print (x,y,z) position
  Eigen::Matrix4d transform;
  group->addFeedbackHandler([&model, &transform](const GroupFeedback& group_fbk)
  {
    Eigen::VectorXd angles = group_fbk.getPosition();
    model.getEndEffector(angles, transform);
    std::cout << std::setw(0.2) <<
       "x " << transform(0,3) <<
      " y " << transform(1,3) <<
      " z " << transform(2,3) << std::endl;
  });
  
  // Control the robot at 100 Hz for 30 seconds
  std::this_thread::sleep_for(std::chrono::seconds(30));
  group->clearFeedbackHandlers();

  return 0;
}
