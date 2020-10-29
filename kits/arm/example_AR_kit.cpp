/*
 * AR Kit Example
 * Currently assumes 6 degrees of freedom in you arm!
 * This demo sets up your arm's end effector to mimick the changes in 
 * position and orientation of your mobile IO device.
 */

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <chrono>
#include <iostream>

using namespace hebi;
using namespace experimental; // For all things mobileIO 



Eigen::Matrix3d makeRotationMatrix (hebi::Quaternionf phone_orientation) {
  Eigen::Quaterniond q;
  q.w() = phone_orientation.getW();
  q.x() = phone_orientation.getX();
  q.y() = phone_orientation.getY();
  q.z() = phone_orientation.getZ();
  return q.toRotationMatrix();
}


int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  arm::Arm::Params params;

  // Setup Module Family and Module Names
  params.families_ = {"Arm"};
  params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};
  
  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  params.hrdf_file_ = "kits/arm/hrdf/A-2085-06.hrdf";  

  // Setup Gripper
  // std::shared_ptr<arm::EffortEndEffector<1>> gripper(arm::EffortEndEffector<1>::create(params.families_[0], "gripperSpool").release());
  // params.end_effector_ = gripper;

  // Create the Arm Object
  auto arm = arm::Arm::create(params);
  while (!arm) {
    arm = arm::Arm::create(params);
  }

  // Load the gains file that is approriate to the arm
  arm -> loadGains("kits/arm/gains/A-2085-06.xml");

  /////////////////////////
  //// MobileIO Setup /////
  /////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create(params.families_[0], "mobileIO");

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Home Position\nB3 - AR Control Mode\n"
                  "B6 - Grav Comp Mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup state variable for mobile device
  auto last_mobile_state = mobile->getState();


  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  // Different modes it can be in (when in none, then automatic grav_comp)
  bool softstart = true;
  bool ar_mode = false;
  
  // Make sure we softstart the arm first.
  Eigen::VectorXd home_position(arm -> robotModel().getDoFCount());
  home_position << 0, M_PI/3, 2*M_PI/3, 5*M_PI/6, -M_PI/2, 0; // Adjust depending on your DoFs

  // Command the softstart to home position
  arm -> update();
  arm -> setGoal(arm::Goal::createFromPosition(4, home_position)); // take 4 seconds
  arm -> send();

  // Get the cartesian position and rotation matrix @ home position
  Eigen::Vector3d xyz_home; 
  Eigen::Matrix3d rot_home;
  arm -> FK(home_position, xyz_home, rot_home); 

  // Set up states for the mobile device
  Eigen::Vector3d xyz_phone_init;
  Eigen::Matrix3d rot_phone_init;

  // Target variables
  Eigen::VectorXd target_joints(arm -> robotModel().getDoFCount());

  while(arm->update())
  {

    if (softstart) {
      // End softstart when arm reaches its homePosition
      if (arm -> atGoal()){
        mobile -> sendText("Softstart Complete!");
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

    // Button B1 - Return to home position
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn) {
        ar_mode = false;
        arm -> setGoal(arm::Goal::createFromPosition(4, home_position));
    }

    // Button B3 - Start AR Control
    if (diff.get(3) == MobileIODiff::ButtonState::ToOn) {
      xyz_phone_init << mobile -> getLastFeedback().mobile().arPosition().get().getX(),
                        mobile -> getLastFeedback().mobile().arPosition().get().getY(),
                        mobile -> getLastFeedback().mobile().arPosition().get().getZ();
      std::cout << xyz_phone_init << std::endl;
      rot_phone_init = makeRotationMatrix(mobile -> getLastFeedback().mobile().arOrientation().get());
      ar_mode = true;
    }

    // Button B6 - Grav Comp Mode
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
      // Get the latest mobile position and orientation
      Eigen::Vector3d xyz_phone;
      xyz_phone << mobile->getLastFeedback().mobile().arPosition().get().getX(),
                   mobile->getLastFeedback().mobile().arPosition().get().getY(),
                   mobile->getLastFeedback().mobile().arPosition().get().getZ();
      auto rot_phone = makeRotationMatrix(mobile->getLastFeedback().mobile().arOrientation().get());

      // Calculate new targets
      Eigen::Vector3d xyz_scale;
      xyz_scale << 1, 1, 2;
      Eigen::Vector3d xyz_target = xyz_home + (0.75 * xyz_scale.array() *
                                (rot_phone_init.transpose() * (xyz_phone - xyz_phone_init)).array()).matrix();
      Eigen::Matrix3d rot_target = rot_phone_init.transpose() * rot_phone * rot_home;

      // Calculate new arm joint angle targets
      target_joints = arm -> solveIK(arm -> lastFeedback().getPosition(),
                                              xyz_target, 
                                              rot_target);

      // Create and send new goal to the arm
      arm -> setGoal(arm::Goal::createFromPosition(target_joints));
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
