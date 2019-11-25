/*
 * TODO: Change this:
 * This file runs a teach_repeat program with a 6-DoF Arm, allowing you to 
 * physically set waypoints for the arm to then move through when you enter
 * playback mode. This is an example of the arm's ability to get accurate 
 * feedback about its position and replay that for the user. Take note, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>
#include <iostream>

using namespace hebi;
using namespace hebi::experimental; // For all things mobileIO 


double currentTime(std::chrono::steady_clock::time_point& start) {
  return (std::chrono::duration<double>(std::chrono::steady_clock::now() - start)).count();
}

int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {"Arm Example"};
  params.names_ = {"Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"};
  
  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  // Make sure you are running this from the correct directory!
  params.hrdf_file_ = "kits/hrdf/6-dof_arm.hrdf";  

  // Setup Gripper
  std::shared_ptr<arm::EffortEndEffector<1>> gripper(arm::EffortEndEffector<1>::create("Arm Example", "Spool").release());
  params.end_effector_ = gripper;

  // Setup Time Variables
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> arm_time = std::chrono::steady_clock::now() - start_time;
  double arm_start_time = arm_time.count();
    
  // Create the Arm Object
  auto arm = arm::Arm::create(arm_start_time, params);




  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("HEBIH", "Mobile IO");

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Setup instructions for display
  std::string instructions;
  // instructions = "Still needs to be done";
  instructions = ("B1 - Home Position\nB2 - Lock Orientation\n"
                  "B3 - AR Control\nB5 - Lock Position\n"
                  "B4 - Grav comp mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup state variable for mobile device
  auto last_mobile_state = mobile->getState();




  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Different modes it can be in (ar_mode, grav_comp_mode)
  bool ar_mode = false;

  // Make sure we softstart the arm first.
  bool softstart = true;
  Eigen::VectorXd home_position(arm -> robotModel().getDoFCount());
  home_position << 0, M_PI/3, 2*M_PI/3, 5*M_PI/6, -M_PI/2, 0;

  Eigen::Vector3d xyz_base; // the new cartesion position
  Eigen::Matrix3d orient_base;

  // Get the initial cartesian position for the arm end effector @ home position
  arm -> FK6Dof(home_position, xyz_base, orient_base); 
  // Eigen::VectorXd zeros = Eigen::VectorXd::Constant(arm -> robotModel().getDoFCount(), 0);
  
  // Command the softstart to home position
  arm -> update(currentTime(start_time));
  arm -> setGoal(arm::Goal(4, home_position)); // take 4 seconds
  arm -> send();

  // Get initial state for the mobile device
  // auto base_phone_pos = mobile -> getPosition();
  Eigen::Vector3d base_phone_pos;
  // base_phone_pos << mobile -> getPosition().getX(),
  //                   mobile -> getPosition().getY(),
  //                   mobile -> getPosition().getZ();

  // TODO: Why is the construction of the Quaternion different from above?
  auto base_phone_ori = mobile -> getOrientation();
  Eigen::Quaterniond q_init;
  q_init.w() = base_phone_ori.getW();
  q_init.x() = base_phone_ori.getX();
  q_init.y() = base_phone_ori.getY();
  q_init.z() = base_phone_ori.getZ();
  auto rot_init = q_init.toRotationMatrix();

  Eigen::MatrixXd rot_target_init(3,3);
  rot_target_init << -1,0,0, 0,1,0, 0,0,-1;  // This should happen at every reset

  // Lock variables
  bool lock_orient = false;
  bool lock_pos = false;

  while(arm->update(currentTime(start_time)))
  {

    if (softstart) {
      // End softstart when arm reaches its homePosition
      if (arm -> atGoal()){
        printf("Softstart complete.\n");
        softstart = false; 
        // base_phone_pos = mobile -> getPosition();
        continue;
      }
      arm -> send();

      // Stay in softstart, don't do any other behavior
      continue;
    } 

    // Get latest mobile_state
    auto mobile_state = mobile->getState();
    MobileIODiff diff(last_mobile_state, mobile_state);

    // Update gripper state as well
    gripper -> getState();

    // Set the gripper value
    double open_val = (mobile_state.getAxis(3) * 2)-1; // makes this range between -2 and 1
    // if (!playback) {
    gripper -> setCommand(0, open_val);
    // }

    // Button B1 - Return to home position
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn) {
        ar_mode = false;
        arm -> setGoal(arm::Goal(4, home_position));
    }

    // Button B2 - Lock Orientation
    if (diff.get(2) == MobileIODiff::ButtonState::ToOn) {
      lock_orient = true;
      lock_pos = false;
      mobile -> sendText("Locked Orientation.");
    }

    // Button B3 - Start AR Control
    if (diff.get(3) == MobileIODiff::ButtonState::ToOn) {
      base_phone_pos << mobile -> getPosition().getX(),
                        mobile -> getPosition().getY(),
                        mobile -> getPosition().getZ();
      base_phone_ori = mobile -> getOrientation();
      lock_orient = false;
      lock_pos = false;
      ar_mode = true;
    }

    // Button B4 - Lock Position
    if (diff.get(4) == experimental::MobileIODiff::ButtonState::ToOn) {
      lock_orient = false;
      lock_pos = true;
      mobile -> sendText("Locked Position.");
    }


    if (diff.get(6) == experimental::MobileIODiff::ButtonState::ToOn) {
      arm -> cancelGoal();
      ar_mode = false;
    }

    // Button B8 - End Demo
    if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn) {
      // Clear MobileIO text
      mobile -> clearText();
      return 1;
    }

    if (ar_mode) {
      Eigen::Vector3d phone_pos;
      phone_pos << mobile -> getPosition().getX(),
                   mobile -> getPosition().getY(),
                   mobile -> getPosition().getZ();
      auto phone_ori = mobile -> getOrientation();

      // Calculate change in phone orientation
      Eigen::Quaterniond q;
      q.w() = phone_ori.getW();
      q.x() = phone_ori.getX();
      q.y() = phone_ori.getY();
      q.z() = phone_ori.getZ();
      auto rot_matrix = q.toRotationMatrix();

      Eigen::Vector3d xyz_scale;
      xyz_scale << -1, -1, -2;

      // Calculate new targets
      auto rot_target = rot_init.transpose() * q.toRotationMatrix() * rot_target_init;
      auto xyz_target = xyz_base + (0.75 * xyz_scale.array() *
                                (rot_init.transpose() * 
                                (base_phone_pos - phone_pos)).array()).matrix();



  


      // auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
      //                                         xyz_target,
      //                                         // xyz_base,
      //                                         rot_target);
      //                                         // orient_base);  



      // if orientation lock
      if (lock_orient) {
        auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
                                                xyz_target,
                                                // xyz_base,
                                                // rot_target);
                                                orient_base);  
        printf("Orientation is locked so we're in here.\n");
        arm -> setGoal(arm::Goal(target_joints));
      }
      // if position lock
      else if (lock_pos) {
        auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
                                                // xyz_target,
                                                xyz_base,
                                                rot_target);
                                                // orient_base);         
        printf("Position is locked so we're in here.\n");
        arm -> setGoal(arm::Goal(target_joints));
      }
      else {
        auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
                                                xyz_target,
                                                // xyz_base,
                                                rot_target);
                                                // orient_base);
        printf("I'm doing normal things\n");
        // std::cout << target_joints << std::endl;
        arm -> setGoal(arm::Goal(target_joints));
      }

      // Send new goal to the arm
      // arm -> setGoal(arm::Goal(target_joints));
    }
   

    // Update mobile device to the new last_state
    last_mobile_state = mobile_state;

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}





// if (ar_mode) {
//       Eigen::Vector3d phone_pos;
//       phone_pos << mobile -> getPosition().getX(),
//                    mobile -> getPosition().getY(),
//                    mobile -> getPosition().getZ();
//       auto phone_ori = mobile -> getOrientation();

//       // Calculate change in phone orientation
//       Eigen::Quaterniond q;
//       q.w() = phone_ori.getW();
//       q.x() = phone_ori.getX();
//       q.y() = phone_ori.getY();
//       q.z() = phone_ori.getZ();
//       auto rot_matrix = q.toRotationMatrix();

//       Eigen::Vector3d xyz_scale;
//       xyz_scale << -1, -1, -2;

//       // Calculate new targets
//       auto rot_target = rot_init.transpose() * q.toRotationMatrix() * rot_target_init;
//       auto xyz_target = xyz_base + (0.75 * xyz_scale.array() *
//                                 (rot_init.transpose() * 
//                                 (base_phone_pos - phone_pos)).array()).matrix();




//       // if orientation lock
//       if (lock_orient) {
//         auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
//                                                 xyz_target,
//                                                 // xyz_base,
//                                                 // rot_target);
//                                                 orient_base);  
//         printf("Orientation is locked so we're in here.\n");
//         arm -> setGoal(arm::Goal(target_joints));
//       }
//       // if position lock
//       else if (lock_pos) {
//         auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
//                                                 // xyz_target,
//                                                 xyz_base,
//                                                 rot_target);
//         printf("Position is locked so we're in here.\n");
//                                                 // orient_base);         
//         arm -> setGoal(arm::Goal(target_joints));
//       }
//       else {
//         auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
//                                                 xyz_target,
//                                                 // xyz_base,
//                                                 rot_target);
//                                                 // orient_base);
//         printf("I'm doing normal things\n");
//         // std::cout << target_joints << std::endl;
//         arm -> setGoal(arm::Goal(target_joints));
//       }

//       // Send new goal to the arm
//       // arm -> setGoal(arm::Goal(target_joints));
//     }