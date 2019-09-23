/**
 * This file demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
// #include "util/grav_comp.hpp"
// #include "arm_api/arm.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include <chrono>
#include <ctime>
#include <thread>
// #include <stdio.h>
// #include <time.h>



int main(int argc, char* argv[])
{
  // Note: this demo is written for a simple 3DOF arm with the kinematics
  // given below.  You can adapt to other systems by creating the correct
  // robot_model object and mass vector.
  // std::unique_ptr<hebi::ArmContainer> arm = hebi::ArmContainer::create3Dof();
  /////////////////
  // hebi::Lookup lookup;
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow", "Wrist1", "Wrist2"};//, "Wrist 2", "Wrist 3"};

  // std::shared_ptr<hebi::Group> group = lookup.getGroupFromNames(family, names);
  // if (!group)
  //   {
  //     std::cout << "Could not find arm group - check names!" << std::endl;
  //   }
  // else {
  //   printf("Found the arm! Let's keep moving...\n");
  // }



  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;
  std::unique_ptr<hebi::robot_model::RobotModel> model(new hebi::robot_model::RobotModel());

  // The following components must be added in the order of the kinematic
  model -> addActuator(ActuatorType::X8_9);
  model -> addBracket(BracketType::X5HeavyRightInside);
  model -> addActuator(ActuatorType::X8_16);
  model -> addLink(LinkType::X5, 0.3, M_PI);
  model -> addActuator(ActuatorType::X8_9);
  model -> addLink(LinkType::X5, 0.3, 0);
  model -> addActuator(ActuatorType::X5_9);
  model -> addBracket(BracketType::X5LightRight);
  model -> addActuator(ActuatorType::X5_4);
  model -> addBracket(BracketType::X5LightLeft);
  model -> addActuator(ActuatorType::X5_4);

  // Set up time variables
  // start time is of the type: std::chrono::time_point<std::chrono::steady_clock> 
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_from_start = std::chrono::steady_clock::now() - start_time;
  double arm_start_time = time_from_start.count();
  
  // Create the Arm
  auto arm = hebi::arm::Arm::create(arm_start_time, family, names, std::move(model));

  // Set up control variables
  int count = 0;
  int num_wp = 2;
  Eigen::VectorXd positions(5);
  Eigen::VectorXd velocities(5);
  Eigen::VectorXd accels(5);
  Eigen::MatrixXd positionsM(5,num_wp);
  Eigen::MatrixXd velocitiesM(5,num_wp);
  Eigen::MatrixXd accelsM(5,num_wp);
  Eigen::VectorXd times(num_wp);


  // Old while loop:
  // while(arm->update( (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time)).count() )) 
  
  // New while Loop 
  while(arm->update(arm->currentTime(start_time)))
  {

    auto& lastfb = arm->lastFeedback();

    // printf("%ld\n", lastfb[1]);
    // std::cout << lastfb.getPosition() << std::endl;

    if (count == 50) {
      positionsM << 0, 0, 0, 0,0,
                    M_PI/4, M_PI/4, M_PI/2, M_PI/4, M_PI/2;
                    // -M_PI/4, M_PI/2, 3*M_PI/4, -M_PI/4, M_PI/4;
      velocitiesM << 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0;
      accelsM << 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0;
      times << 5, 15;//, 20;
      arm -> setGoal(hebi::arm::Goal(times, positionsM, velocitiesM, accelsM));
      // Is this time when you command the goal, or when you have to reach the goal?
    }

    
    count++;
    arm->send();
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






  // model -> addActuator(ActuatorType::X8_9);
  // model -> addBracket(BracketType::X5HeavyRightInside);
  // model -> addActuator(ActuatorType::X8_16);
  // model -> addLink(LinkType::X5, 0.3, M_PI);
  // model -> addActuator(ActuatorType::X8_9);
  // model -> addLink(LinkType::X5, 0.3, 0);
  // model -> addActuator(ActuatorType::X5_9);
  // model -> addBracket(BracketType::X5LightRight);










//   // std::shared_ptr<hebi::Group> group = lookup.getGroupFromNames(family, names);
//   // if (!group)
//   //   {
//   //     std::cout << "Could not find arm group - check names!" << std::endl;
//   //   }

//   // // hebi::arm::ArmKinematics arm_kin = hebi::arm::ArmKinematics(model);
//   // hebi::arm::ArmKinematics arm_kin(model);

//   // Eigen::VectorXd home_pos(model.getDoFCount());
//   // home_pos << 0, 3*M_PI/4, 3*M_PI/4, 0;

//   // // auto start_time = std::chrono::system_clock::now();
//   // time_t start_time = time(0);
//   // std::unique_ptr<hebi::arm::Arm> arm = hebi::arm::Arm::createArm(family, names, 
//   //                                                   home_pos, arm_kin, 0);


//   /*
//   What we're doing is:
//   - while arm is working and responding
//   - take current position
//   - get the efforts required at this point to negate gravity
//   - send them
//   - continue this indefinitely until we quit the application
//   */



//   hebi::GroupCommand cmd(arm -> getGroup() -> size());
//   // Eigen::Vector3d gravity(0, 0, -1); // (x,y,z)
//   // Eigen::VectorXd masses;
//   // masses.resize(arm -> getKinematics().getModel().getFrameCount(HebiFrameTypeCenterOfMass));
//   // arm -> getKinematics().getModel().getMasses(masses);
//   int count = 0;

// /* Main Loop */
//   while(arm -> update(difftime(time(0), start_time)))
//   {
//      // fb = arm -> getLastFeedback();
//     // Eigen::VectorXd effort = arm -> getKinematics().gravCompEfforts(arm -> getLastFeedback());
//     // cmd.setEffort(effort);
//     // arm -> getGroup() -> sendCommand(cmd);
//     // std::cout << difftime(time(0), start_time);
//     if (count == 500){
//       // std::cout << "I'm in here wassup";
//       // Eigen::VectorXd positions(model.getDoFCount());
//       // positions << M_PI/4, M_PI/2, M_PI/2, M_PI/2;
//       // arm -> getTrajectory().replan(difftime(time(0), start_time),
//                                     // arm->getLastFeedback(),
//                                     // positions);

//       Eigen::MatrixXd positions(4,1);
//       // Eigen::VectorXd positions(4);
//       positions << M_PI/4, M_PI/2, M_PI/2, M_PI/2,
//                    0, 3*M_PI/4, M_PI/2, 0;
                   
//                    // -M_PI/4, M_PI/2, M_PI/2, -M_PI/2;
//       // Eigen::VectorXd times(4);
//       // times << 0, 5, 10, 15;
//       Eigen::VectorXd times(1);
//       times << 3;
//       arm -> getTrajectory().replan(difftime(time(0), start_time),
//                                     arm->getLastFeedback(),
//                                     positions, times);




//     }

//     count++;
//     // arm -> getTrajectory().replan()
//     // arm -> update(difftime(time(0), start_time));
//     // std::chrono"
//   }