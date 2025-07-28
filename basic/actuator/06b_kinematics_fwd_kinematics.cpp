#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "hebi_charts.hpp"

#include <Eigen/Dense>

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**)
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

  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::Window window;
    auto chart = window.addChart3d();
    window.show();
    // Get frame count; add one for base frame added after FK call below)
    auto num_frames = model->getFrameCount(robot_model::FrameType::Output) + 1;
    std::vector<hebi::charts::Chart3dTriad> triads;
    for (size_t i = 0; i < num_frames; ++i)
      triads.push_back(chart.addTriad(0.075));
    // We set the loop runs at 10Hz; so, this runs for 20 seconds:
    for(size_t i = 0; i < 200; i++) {
      if (group->getNextFeedback(group_fbk)){
        Eigen::VectorXd angles = group_fbk.getPosition();
        model->getFK(robot_model::FrameType::Output, angles, transforms);

        //plot frames on a 3d graph
        transforms.emplace(transforms.begin(),Eigen::Matrix<double,4,4>::Identity());
        for(size_t j = 0; j < transforms.size(); ++j) {
          Eigen::Quaterniond q(Eigen::Matrix3d(transforms[j].topLeftCorner(3, 3)));
          triads[j].setOrientation(q.w(), q.x(), q.y(), q.z());
          Eigen::Vector3d xyz = transforms[j].topRightCorner(3, 1);
          triads[j].setTranslation(xyz.x(), xyz.y(), xyz.z());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}