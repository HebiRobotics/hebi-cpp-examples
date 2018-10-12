#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"

using namespace hebi;

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
  auto model =
    robot_model::RobotModel::loadHRDF("hrdf/3-DoF_arm_example.hrdf");
  if (!model)
  {
    std::cout << "Could not load HRDF!" << std::endl;
    return -1;
  }

  // Add a callback function to print (x,y,z) position
  Eigen::Matrix4d transform;
  group->addFeedbackHandler([&model, &transform](const GroupFeedback& group_fbk)
  {
    Eigen::VectorXd angles = group_fbk.getPosition();
    model->getEndEffector(angles, transform);
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
