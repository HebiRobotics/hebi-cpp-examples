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

  // Variable to switch gripper control from mobile_io to trajectory and vice versa
  bool playback = false;


  // last_phone_pos
  auto last_phone_pos = mobile -> getPosition();

  // Make sure we softstart the arm first.
  bool softstart = true;
  bool softsend = true;
  Eigen::VectorXd home_position(arm -> robotModel().getDoFCount());
  Eigen::VectorXd zeros = Eigen::VectorXd::Constant(arm -> robotModel().getDoFCount(), 0);

  home_position << 0, M_PI/3, 2*M_PI/3, 5*M_PI/6, -M_PI/2, 0;
  arm -> update(currentTime(start_time));
  arm -> setGoal(arm::Goal(4, home_position));
  arm -> send();

  while(arm->update(currentTime(start_time)))
  {

    if (softstart) {
      // End softstart when arm reaches its homePosition
      if (arm -> atGoal()){
        printf("Softstart complete\n");
        softstart = false;
        continue;
      }
      arm -> send();

      // Stay in softstart, don't do any other behavior
      continue;
    }


     // Get latest mobile_state
    auto mobile_state = mobile->getState();
    MobileIODiff diff(last_mobile_state, mobile_state);


    // std::cout << mobile -> getLastFeedback().mobile().arPosition().get().getX() << std::endl;
    auto phone_pos = mobile -> getPosition();
    // std::cout << posio.getX() << posio.getY()
    // printf("(%f, %f, %f)\n", posio.getX(), posio.getY(), posio.getZ());

    auto oreo = mobile -> getOrientation();

    // printf("(%f, %f, %f, %f)\n", oreo.getW(), oreo.getX(), oreo.getY(), oreo.getZ());



    // Testing
    gripper->getState();

    // Set the gripper value
    double open_val = (mobile_state.getAxis(3) * 2)-1; // makes this range between -2 and 1
    
    if (!playback) {
      // gripper -> setCommand(0, open_val);
    }
    
    // Buttton B1 - Reset Orientation
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn) {
      // addWaypoint(state, arm -> lastFeedback(), open_val, true);
    }

    // Button B2 - Freeze Position (x,y,z)
    if (diff.get(2) == MobileIODiff::ButtonState::ToOn) {
      // state.waypoints.clear();
    }

    // Button B3 - Freeze Rotation (roll, pitch, yaw)
    if (true) {//(diff.get(3) == MobileIODiff::ButtonState::ToOn) {
      // addWaypoint(state, arm -> lastFeedback(), open_val, false);

      // Take in xyz from the phone;
      // auto phone_pos = mobile -> getPosition();

      hebi::Vector3f phoneDiff(phone_pos.getX() - last_phone_pos.getX(),
                         phone_pos.getY() - last_phone_pos.getY(),
                         phone_pos.getZ() - last_phone_pos.getZ());
      // Take in the current position of the end effector in cartesian space
      Eigen::Vector3d xyz_out;
      Eigen::Matrix3d orient_out;
      auto current_joints = arm -> lastFeedback().getPosition();
      arm -> FK6Dof(current_joints, xyz_out, orient_out); 

      // std::cout << xyz_out << std::endl;

      // Calculate new targets
      // THIS WONT WORK CAUSE YOU NEED THE DIFFERENCE IN THEIR POSITIONS
      Eigen::Vector3d target_xyz;
      float scale = 50;
      target_xyz << xyz_out[0] + phoneDiff.getX() * scale, 
                    xyz_out[1] + phoneDiff.getY() * scale,
                    xyz_out[2] + phoneDiff.getZ() * scale;

      auto target_joints = arm -> solveIK6Dof(current_joints,
                                        target_xyz,
                                        orient_out);

      std::cout << target_joints << std::endl;

      arm -> setGoal(arm::Goal(target_joints));
    



      // So what I really gotta be doing is, upon startup,
      // Note down the initial ipad pos, and I already know the initial pos for arm
      // Then from there, I also compare difference to that base value for both of them
      // And thus, all the reset button ever does is reset the base values so that
      // everything from that point forward is relative to those reference values.



    }

    if (diff.get(6) == experimental::MobileIODiff::ButtonState::ToOn) {
      arm -> cancelGoal();
    }

    // Button B8 - End Demo
    if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn) {
      // Clear MobileIO text
      mobile -> clearText();
      return 1;
    }

    // Update to the new last_state
    last_mobile_state = mobile_state;
    last_phone_pos = phone_pos;

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}

