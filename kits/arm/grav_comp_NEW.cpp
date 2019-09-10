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
// #include "arm_container.hpp"
// #include "arm_api/arm_kinematics.hpp"
#include "arm_api/arm.hpp"
#include "lookup.hpp"
#include <chrono>
#include <ctime>
#include <thread>



int main(int argc, char* argv[])
{
  // Note: this demo is written for a simple 3DOF arm with the kinematics
  // given below.  You can adapt to other systems by creating the correct
  // robot_model object and mass vector.
  // std::unique_ptr<hebi::ArmContainer> arm = hebi::ArmContainer::create3Dof();
/////////////////
  // hebi::Lookup lookup;
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow", "Wrist1"};//, "Wrist 2", "Wrist 3"};


  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;
  hebi::robot_model::RobotModel model;

  model.addActuator(ActuatorType::X8_9);
  model.addBracket(BracketType::X5HeavyRightInside);
  model.addActuator(ActuatorType::X8_16);
  model.addLink(LinkType::X5, 0.3, M_PI);
  model.addActuator(ActuatorType::X8_9);
  model.addLink(LinkType::X5, 0.3, 0);
  model.addActuator(ActuatorType::X5_9);
  model.addBracket(BracketType::X5LightRight);
  // model -> addActuator(ActuatorType::X5_4);
  // model -> addBracket(BracketType::X5LightLeft);
  // model -> addActuator(ActuatorType::X5_4);

 

  // std::shared_ptr<hebi::Group> group = lookup.getGroupFromNames(family, names);
  // if (!group)
  //   {
  //     std::cout << "Could not find arm group - check names!" << std::endl;
  //   }

  // hebi::arm::ArmKinematics arm_kin = hebi::arm::ArmKinematics(model);
  hebi::arm::ArmKinematics arm_kin(model);

  Eigen::VectorXd home_pos(model.getDoFCount());
  home_pos << 0, 3*M_PI/4, 3*M_PI/4, 0;

  // auto start_time = std::chrono::system_clock::now();
  time_t start_time = time(0);
  std::unique_ptr<hebi::arm::Arm> arm = hebi::arm::Arm::createArm(family, names, 
                                                    home_pos, arm_kin, 0);


  /*
  What we're doing is:
  - while arm is working and responding
  - take current position
  - get the efforts required at this point to negate gravity
  - send them
  - continue this indefinitely until we quit the application
  */



  hebi::GroupCommand cmd(arm -> getGroup() -> size());
  // Eigen::Vector3d gravity(0, 0, -1); // (x,y,z)
  // Eigen::VectorXd masses;
  // masses.resize(arm -> getKinematics().getModel().getFrameCount(HebiFrameTypeCenterOfMass));
  // arm -> getKinematics().getModel().getMasses(masses);
  int count = 0;

/* Main Loop */
  while(arm -> update(difftime(time(0), start_time)))
  {
     // fb = arm -> getLastFeedback();
    // Eigen::VectorXd effort = arm -> getKinematics().gravCompEfforts(arm -> getLastFeedback());
    // cmd.setEffort(effort);
    // arm -> getGroup() -> sendCommand(cmd);
    // std::cout << difftime(time(0), start_time);
    if (count == 500){
      // std::cout << "I'm in here wassup";
      // Eigen::VectorXd positions(model.getDoFCount());
      // positions << M_PI/4, M_PI/2, M_PI/2, M_PI/2;
      // arm -> getTrajectory().replan(difftime(time(0), start_time),
                                    // arm->getLastFeedback(),
                                    // positions);

      Eigen::MatrixXd positions(4,1);
      // Eigen::VectorXd positions(4);
      positions << M_PI/4, M_PI/2, M_PI/2, M_PI/2;
                   // 0, 3*M_PI/4, M_PI/2, 0,
                   // -M_PI/4, M_PI/2, M_PI/2, -M_PI/2;
      // Eigen::VectorXd times(4);
      // times << 0, 5, 10, 15;
      Eigen::VectorXd times(1);
      times << 3;
      arm -> getTrajectory().replan(difftime(time(0), start_time),
                                    arm->getLastFeedback(),
                                    positions, times);




    }

    count++;
    // arm -> getTrajectory().replan()
    // arm -> update(difftime(time(0), start_time));
    // std::chrono"
  }

  return 0;
}


  /* METHOD ONE */
  // while(arm -> update(difftime(time(0), start_time)))
  // {
  //    // fb = arm -> getLastFeedback();
  //   // Eigen::VectorXd effort = arm -> getKinematics().gravCompEfforts(arm -> getLastFeedback());
  //   // cmd.setEffort(effort);
  //   // arm -> getGroup() -> sendCommand(cmd);
  //   arm -> update(difftime(time(0), start_time));
  //   // std::chrono"
  // }

  /* METHOD TWO */
  // if (!arm)
  // {
  //   return -1;
  // }
  // // std::cout << "we made it this far";
  // arm -> getGroup() -> addFeedbackHandler(
  //   [&arm, &cmd](const hebi::GroupFeedback& feedback)->void
  //     {
  //       Eigen::VectorXd effort = arm -> getKinematics().gravCompEfforts(arm->getLastFeedback());
  //       cmd.setEffort(effort);
  //       arm -> getGroup() -> sendCommand(cmd);
  //     });

  // std::this_thread::sleep_for(std::chrono::seconds(60));
  // arm -> getGroup() -> clearFeedbackHandlers();




















// #include "group.hpp"
// #include "group_command.hpp"
// #include "group_feedback.hpp"
// #include "arm_api/arm.hpp"
// #include "lookup.hpp"
// // #include "util/grav_comp.hpp"
// // #include "arm_container.hpp"
// #include <chrono>
// #include <thread>




// int main(int argc, char* argv[])
// {

//   std::vector<std::string> family_name = {"Arm Example"};
//   std::vector<std::string> module_names = {"Base", "Shoulder", "Elbow", "Wrist1"};

//   std::unique_ptr<hebi::robot_model::RobotModel> model(
//     new hebi::robot_model::RobotModel());

//   model -> addActuator(ActuatorType::X8_9);
//   model -> addBracket(BracketType::X5HeavyRightInside);
//   model -> addActuator(ActuatorType::X8_16);
//   model -> addLink(LinkType::X5, 0.3, M_PI);
//   model -> addActuator(ActuatorType::X8_9);
//   model -> addLink(LinkType::X5, 0.3, 0);
//   model -> addActuator(ActuatorType::X5_9);
//   model -> addBracket(BracketType::X5LightRight);
//   // model -> addActuator(ActuatorType::X5_4);
//   // model -> addBracket(BracketType::X5LightLeft);
//   // model -> addActuator(ActuatorType::X5_4);

//   ArmKinematics arm_kin = 

//   Eigen::VectorXd home_pos = {0, 3*M_PI/4, 3*M_PI/4, 0};



//   createArm(family_name, module_names, home_pos)





// }



















