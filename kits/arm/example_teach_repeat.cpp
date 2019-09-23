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

struct Waypoint
{
  Eigen::VectorXd _positions;
  Eigen::VectorXd _vels;
  Eigen::VectorXd _accels;
};
  
struct State
{
  int _num_modules;
  std::vector<Waypoint> _waypoints;
  std::vector<bool> _stops;
};

void addWaypoint (State& state, const GroupFeedback& feedback, bool stop) {
  printf("Adding a Waypoint.\n");
  state._waypoints.push_back(Waypoint {feedback.getPosition(),
                                       VectorXd::Constant(state._num_modules, 0),
                                       VectorXd::Constant(state._num_modules, 0)});
  state._stops.push_back(stop);



}

// TODO: Add functionality to make it so that its possible to slide through the different Trajectories as well.
arm::Goal playWaypoints (State& state) {

  // Set up the required variables
  Eigen::MatrixXd targets_pos(state._num_modules, state._waypoints.size());
  Eigen::MatrixXd targets_vels(state._num_modules, state._waypoints.size());
  Eigen::MatrixXd targets_accels(state._num_modules, state._waypoints.size());
  Eigen::VectorXd times(state._waypoints.size());

  // Fill up the matrix
  for (int i = 0; i < state._waypoints.size(); i++)
  {
    // map each waypoint vector to a column in the targets matrix
    targets_pos.col(i) << state._waypoints[i]._positions;
    if (state._stops[i]) {
      targets_vels.col(i) << Eigen::VectorXd::Constant(state._num_modules, 0);
      targets_accels.col(i) << Eigen::VectorXd::Constant(state._num_modules, 0);
    } else {
      targets_vels.col(i) << Eigen::VectorXd::Constant(state._num_modules, std::numeric_limits<double>::quiet_NaN());
      targets_accels.col(i) << Eigen::VectorXd::Constant(state._num_modules, std::numeric_limits<double>::quiet_NaN());
    }
    // times[i] = 3*(i+1); // Each movement will take 3 seconds.
  }

  auto goal = arm::Goal(targets_pos, targets_vels, targets_accels);

  // auto goal = arm::Goal(targets, 
     // Eigen::MatrixXd::Constant(state._num_modules, state._waypoints.size(), 0),
     // Eigen::MatrixXd::Constant(state._num_modules, state._waypoints.size(), 0));
  return goal;
}



int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Setup Module Family and Module Names
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow",
                                    "Wrist1", "Wrist2", "Wrist3"};

  // Read HRDF file to setup a RobotModel object for the 6-DoF Arm
  std::unique_ptr<robot_model::RobotModel> model = 
                      robot_model::RobotModel::loadHRDF("kits/hrdf/6-dof_arm.hrdf");
  //TODO: Check if the HRDF needs to be updated back to something appropriate
  // for the user. AKA back into the form that we ship ou the 6-dofs in.

  // TODO: HRDF description on the Doxygen generated page is incorrectly copy pasted

  // Setup Time Variables
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_from_start = 
                                  std::chrono::steady_clock::now() - start_time;
  double arm_start_time = time_from_start.count();
  
  // Create the Arm Object
  auto arm = arm::Arm::create(arm_start_time, family, names, std::move(model));

  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("HEBI", "Mobile IO");

  std::string instructions;
  instructions = "B1 - Add Waypoint\nB2 - Clear Waypoints\nB5 - Playback Mode\nB6 - Grav Comp Mode\nB8 - Quit\n";
  // Clear any garbage on screen
  mobile -> clearText(); 

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup instructions
  auto last_mobile_state = mobile->getState();

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Teach Repeat Variables
  State state;
  state._num_modules = arm -> robotModel().getDoFCount();

  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////


  // instructions = "B1 - Add Waypoint\nB2 - Clear Waypoints\nB5 - Play Waypoints\nB6 - Return to Grav Comp\n";


  while(arm->update(arm->currentTime(start_time)))
  {
     // Get latest mobile_state
    auto mobile_state = mobile->getState();
    MobileIODiff diff(last_mobile_state, mobile_state);

    // Buttton B1 - Home Position
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn) {
      addWaypoint(state, arm -> lastFeedback(), true);
    }

    // Button B2 - Clear Waypoints
    if (diff.get(2) == MobileIODiff::ButtonState::ToOn) {
      state._waypoints.clear();
      state._stops.clear();
    }

    // Button B3 - Through Waypoint
    if (diff.get(3) == MobileIODiff::ButtonState::ToOn) {
      addWaypoint(state, arm -> lastFeedback(), false);
    }


    // Button B5 - Switch to replay mode
    if (diff.get(5) == MobileIODiff::ButtonState::ToOn) {
      if (state._waypoints.size() == 0){
        printf("You have not added any Waypoints!\n");
      } 
      else {
        const arm::Goal trajectory = playWaypoints(state);
        arm -> setGoal(trajectory);       
      }

    }

    // Button B6 - Grav Comp Mode
    if (diff.get(6) == MobileIODiff::ButtonState::ToOn) {
      // cancel any goal that is set, returning arm into gravComp mode
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

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}



