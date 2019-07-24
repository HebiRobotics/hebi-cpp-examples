#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "plot_functions.h"

using namespace hebi;

namespace plt = matplotlibcpp;

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
  hebi::robot_model::Matrix4dVector transforms;
  GroupFeedback group_fbk(group->size());
  for(size_t i = 0; i < 50; i++) {
    if (group->getNextFeedback(group_fbk)){
      Eigen::VectorXd angles = group_fbk.getPosition();
      model->getFK(HebiFrameTypeOutput, angles, transforms);

      //plot frames on a 3d graph
      transforms.emplace(transforms.begin(),Eigen::Matrix<double,4,4>::Identity());
      std::vector<std::vector<double>> lines_x;
      std::vector<std::vector<double>> lines_y;
      std::vector<std::vector<double>> lines_z;

      plt::clf();
      for(size_t j = 0; j < transforms.size(); ++j) {
	plot_3dtriad(transforms[j],&lines_x,&lines_y,&lines_z, static_cast<bool>(j));
      }
      plt::pause(1);
    }
  }
  
  return 0;
}
