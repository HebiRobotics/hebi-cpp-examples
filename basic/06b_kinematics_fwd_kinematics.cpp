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
  std::shared_ptr<Group> group = lookup.getGroupFromNames({ "Hi5 Generator" }, { "Wrist", "Shoulder", "Elbow" });

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
      transforms.emplace(transforms.begin(),Eigen::Matrix<double,4,4>::Identity());
      std::vector<std::vector<double>> lines_x;
      lines_x.resize(3*transforms.size());
      std::vector<std::vector<double>> lines_y;
      lines_y.resize(3*transforms.size());
      std::vector<std::vector<double>> lines_z;
      lines_z.resize(3*transforms.size());

      plt::clf();
      for(size_t j = 0; j < transforms.size(); ++j) {
	Eigen::Matrix<double,4,2> line_x, line_y, line_z;
	line_x << 0, .1,
	          0, 0,
		  0, 0,
		  1, 1;
	line_y << 0, 0,
	          0, .1,
		  0, 0,
		  1, 1;
	line_z << 0, 0,
	          0, 0,
		  0, .1,
		  1, 1;
	line_x = transforms[j]*line_x;
	line_y = transforms[j]*line_y;
	line_z = transforms[j]*line_z;
	lines_x[j * 3] = { line_x(0,0),line_x(0,1) };
	lines_x[j*3+1] = { line_y(0,0),line_y(0,1) };
	lines_x[j*3+2] = { line_z(0,0),line_z(0,1) };

        lines_y[j * 3] = { line_x(1,0),line_x(1,1) };
	lines_y[j*3+1] = { line_y(1,0),line_y(1,1) };
	lines_y[j*3+2] = { line_z(1,0),line_z(1,1) };

        lines_z[j * 3] = { line_x(2,0),line_x(2,1) };
	lines_z[j*3+1] = { line_y(2,0),line_y(2,1) };
	lines_z[j*3+2] = { line_z(2,0),line_z(2,1) };

        plt::plot_3dline(lines_x[j*3],lines_y[j*3],lines_z[j*3],j==0?"k":"r");
        plt::plot_3dline(lines_x[j*3+1],lines_y[j*3+1],lines_z[j*3+1],j==0?"k":"g");
        plt::plot_3dline(lines_x[j*3+2],lines_y[j*3+2],lines_z[j*3+2],j==0?"k":"b");


      }
      plt::pause(1);
    }
  }
  
  // Control the robot at 100 Hz for 30 seconds
  //std::this_thread::sleep_for(std::chrono::seconds(30));
  group->clearFeedbackHandlers();

  return 0;
}
