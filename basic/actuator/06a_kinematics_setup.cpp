/**
 * Set up robot kinematics based on HEBI Robot Definition Format (HRDF) file, as
 * well as through robot model class.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * HEBI Robotics
 * October 2018
 */

#include <iostream>
#include "robot_model.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

using namespace hebi;
using ActuatorType = robot_model::ActuatorType;
using BracketType = robot_model::BracketType;
using LinkType = robot_model::LinkType;

int main()
{
  // Method #1:
  // Load the kinematics from an HRDF file
  std::unique_ptr<robot_model::RobotModel> model_from_hrdf =
    robot_model::RobotModel::loadHRDF("hrdf/3-DoF_arm_example.hrdf");
  if (!model_from_hrdf)
  {
    std::cout << "Could not load HRDF!" << std::endl;
    return -1;
  }

  // Display basic kinematics information
  std::cout << "Robot model loaded from HRDF has "
            << model_from_hrdf->getDoFCount()
            << " degrees of freedom." << std::endl;

  // Method #2:
  // Create a simple kinematic description of the arm in code
  robot_model::RobotModel model_from_code;
  model_from_code.addActuator(ActuatorType::X5_9);
  model_from_code.addBracket(BracketType::X5HeavyLeftOutside);
  model_from_code.addActuator(ActuatorType::X5_9);
  model_from_code.addLink(LinkType::X5, 0.325, M_PI);
  model_from_code.addActuator(ActuatorType::X5_4);
  model_from_code.addLink(LinkType::X5, 0.325, 0);

  // Display basic kinematics information
  std::cout << "Robot model generated in code has "
            << model_from_code.getDoFCount()
            << " degrees of freedom." << std::endl;

  return 0;
}
