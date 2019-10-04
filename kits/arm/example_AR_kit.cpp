/*
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
  std::unique_ptr<MobileIO> mobile = MobileIO::create("HEBI", "Mobile IO");

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Add stop WP\nB2 - Clear waypoints\n"
                  "B3 - Add through WP\nB5 - Playback mode\n"
                  "B6 - Grav comp mode\nB8 - Quit\n");

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
  arm -> setGoal(arm::Goal(4, home_position));
  arm -> send();

  // Get initial state for the mobile device
  // auto base_phone_pos = mobile -> getPosition();
  Eigen::Vector3d base_phone_pos;
  base_phone_pos << mobile -> getPosition().getX(),
                    mobile -> getPosition().getY(),
                    mobile -> getPosition().getZ();
  auto base_phone_ori = mobile -> getOrientation();
  Eigen::Quaterniond q_init;
  q_init.w() = base_phone_ori.getW();
  q_init.x() = base_phone_ori.getX();
  q_init.y() = base_phone_ori.getY();
  q_init.z() = base_phone_ori.getZ();
  auto rot_init = q_init.toRotationMatrix();

  Eigen::MatrixXd rot_target_init(3,3);
  rot_target_init << -1,0,0, 0,1,0, 0,0,-1;  // This should happen at every reset


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
    
    // Buttton B1 - Reset Orientation
    // Button B2 - Freeze Position (x,y,z)

    // Button B3 - Freeze Rotation (roll, pitch, yaw)
    if (diff.get(3) == MobileIODiff::ButtonState::ToOn) {
      base_phone_pos << mobile -> getPosition().getX(),
                        mobile -> getPosition().getY(),
                        mobile -> getPosition().getZ();
      base_phone_ori = mobile -> getOrientation();
      ar_mode = true;
    }

    if (ar_mode) {
      // auto phone_pos = mobile -> getPosition();
      Eigen::Vector3d phone_pos;
      phone_pos << mobile -> getPosition().getX(),
                   mobile -> getPosition().getY(),
                   mobile -> getPosition().getZ();
      auto phone_ori = mobile -> getOrientation();

      // Calculate change in phone position
      // hebi::Vector3f phone_diff(phone_pos.getX() - base_phone_pos.getX(),
      //                           phone_pos.getY() - base_phone_pos.getY(),
      //                           phone_pos.getZ() - base_phone_pos.getZ());

      // Calculate change in phone orientation
      Eigen::Quaterniond q;
      q.w() = phone_ori.getW();
      q.x() = phone_ori.getX();
      q.y() = phone_ori.getY();
      q.z() = phone_ori.getZ();
      auto rot_matrix = q.toRotationMatrix();

      auto rot_target = rot_init.transpose() * q.toRotationMatrix() * rot_target_init;


      Eigen::Vector3d xyz_scale;
      xyz_scale << -1, -1, -2;
      auto xyz_target = xyz_base + (0.75 * xyz_scale.array() * (rot_init.transpose() * (base_phone_pos - phone_pos)).array()).matrix();
      //xyz_target_init is basically the xyz of where we started
      // getting rid of phone_control_scale
      // xyz_scale_array

// auto xyz_target = xyz_target_init + (phone_control_scale * xyz_scale.array() * (r_init.transpose() * (xyz_phone_new - xyz_init)).array()).matrix();


      // Calculate new targets
      // Eigen::Vector3d xyz_target;
      float scale = 0.75;
      // xyz_target << xyz_base[0] + phone_diff.getX() * scale,
      //               xyz_base[1] + phone_diff.getY() * scale,
      //               xyz_base[2] + phone_diff.getZ() * 2;


      // printf("(%f, %f, %f)\n", xyz_base[0], xyz_base[1], xyz_base[2]);
      // printf("(%f, %f, %f)\n", phone_diff.getX(), phone_diff.getY(), phone_diff.getZ());
      // printf("(%f, %f, %f)\n\n", xyz_target[0], xyz_target[1], xyz_target[2]);

      // Set the new target joint values for the arm
      auto target_joints = arm -> solveIK6Dof(arm -> lastFeedback().getPosition(),
                                              xyz_target,
                                              // xyz_base,
                                              rot_target);
                                              // orient_base);

      // Send new goal to the arm
      arm -> setGoal(arm::Goal(target_joints));
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

    // Update mobile device to the new last_state
    last_mobile_state = mobile_state;

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}

