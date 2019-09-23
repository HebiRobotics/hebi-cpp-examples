/**
 * This file is a barebones skeleton of how to setup an arm for use.
 * It demonstrates gravity compensation behavior by commanding torques
 * equal to the force from gravity on the links and joints of an arm.
 * Note that this only approximately balances out gravity, as imperfections in
 * the torque sensing and modeled system can lead to "drift".  Also, the
 * particular choice of PID control gains can affect the performance of this
 * demo.
 */

// #include "lookup.hpp"
// #include "group.hpp"
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
  
enum class Mode { Training, Playback };

struct State
{
  int _num_modules;
  std::vector<Waypoint> _waypoints;
  Mode _mode { Mode::Training };
};


// TODO: Should this be static?
void addWaypoint (State& state, const GroupFeedback& feedback) {
  printf("Adding a Waypoint.\n");

  state._waypoints.push_back(Waypoint {feedback.getPosition(),
                                       VectorXd::Constant(state._num_modules, 0),
                                       VectorXd::Constant(state._num_modules, 0)});
}

// TODO: Add functionality to make it so that its possible to slide through the different Trajectories as well.
arm::Goal playWaypoints (State& state) {
// void playWaypoints (State& state) {
  // This function will start wherever you are, then go through the wayooints
  // So plan goal to next waypoint, then when you get there, you keep going.
  // OOOORRRR
  // What we do is that we construct a matrix with all the positions we need to go to.
  // Then we just have it play through all of them.

  int wp_time = 2; // 3 Seconds to go between the different waypoints

  // Set up the required variables
  // int num_wp = state._waypoints.size();
  Eigen::MatrixXd targets(state._num_modules, state._waypoints.size());
  Eigen::VectorXd times(state._waypoints.size());

  // Fill up the matrix
  for (int i = 0; i < state._waypoints.size(); i++)
  {
    targets.col(i) << state._waypoints[i]._positions;
    
    if (i == 0) {
      times[i] = wp_time;
    } else {
      times[i] = times[i-1] + wp_time;
    }

    // targets.col(i) = Eigen::Map<Eigen::VectorXd>(state._waypoints[i], state._num_modules);
    // TODO: Make this more efficient by getting this Map function to work
    // for (int n = 0; n < state._num_modules; n++)
    // {
    //   targets.col(i) << state._waypoints[i][n];
    // }
  }

  //TODO: construct this vector better as well:
  // std::cout << state._waypoints.size() <<std::endl;

  // std::cout << times << std::endl;
  std::cout << targets << std::endl;
  std::cout << "Now for times: \n";
  std::cout << times << std::endl;
  // return arm::Goal(times, targets);

  auto wahoo = arm::Goal(targets, 
                         Eigen::MatrixXd::Constant(state._num_modules, state._waypoints.size(), 0),
                         Eigen::MatrixXd::Constant(state._num_modules, state._waypoints.size(), 0));
  return wahoo;
}






int main(int argc, char* argv[])
{
  //////////////////////////
  ///// Arm Setup //////////
  //////////////////////////

  // Setup Module Family and Module Names
  std::vector<std::string> family = {"Arm Example"};
  std::vector<std::string> names = {"Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"};

  // Setup a RobotModel object for the 6-DoF Arm
  // This can alternatively be done using a HRDF file 
  // Check docs.hebi.us to learn more about HRDFs and how to load them

  // std::unique_ptr<robot_model::RobotModel> model = 
                      // robot_model::RobotModel::loadHRDF("hrdf/6-dof_arm.hrdf");
  //TODO: Check if the HRDF needs to be updated back to something appropriate
  // for the user. AKA back into the form that we ship ou the 6-dofs in.

  // TODO: HRDF description on the Doxygen generated page is incorrectly copy pasted

  using ActuatorType = robot_model::RobotModel::ActuatorType;
  using BracketType = robot_model::RobotModel::BracketType;
  using LinkType = robot_model::RobotModel::LinkType;
  std::unique_ptr<robot_model::RobotModel> model(new robot_model::RobotModel());

  // // TODO TOCONTINUE: Change this to be an HRDF
  // // Update the documentation accordingly...?
  // // Then Switch over to doing this in teach repeat

  // Create the Arm
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

  // Setup Time Variables
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_from_start = std::chrono::steady_clock::now() - start_time;
  double arm_start_time = time_from_start.count();
  
  // Create the Arm Object
  auto arm = arm::Arm::create(arm_start_time, family, names, std::move(model));


  //////////////////////////
  //// MobileIO Setup //////
  //////////////////////////

  // Create the MobileIO object
  std::unique_ptr<MobileIO> mobile = MobileIO::create("HEBI", "Mobile IO");

  std::string instructions;
  instructions = "B1 - Home Position\nB2 - Waypoint 1\nB5 - Waypoint 2\nB6 - Grav Comp Mode\nB8 - End Demo\n";

  // Clear any garbage on screen
  mobile -> clearText(); 

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup instructions
  auto last_mobile_state = mobile->getState();

  /////////////////////////////
  // Control Variables Setup //
  /////////////////////////////

  // Single Waypoint Vectors
  // auto num_joints = arm -> robotModel().getDoFCount();
  // Eigen::VectorXd positions(num_joints);
  double single_time;

  // Time Vector for Waypoints
  int num_wp = 1; 
  Eigen::VectorXd times(num_wp);

  // Teach Repeat Variables
  State state;
  state._num_modules = arm -> robotModel().getDoFCount();
  // Waypoint wp;
  // wp._positions << 0, 0, 0, 0, 0, 0;
  // state._waypoints.push_back(wp); // This would be where you add current position



  //////////////////////////
  //// Main Control Loop ///
  //////////////////////////

  while(arm->update(arm->currentTime(start_time)))
  {
     // Get latest mobile_state
    auto mobile_state = mobile->getState();
    MobileIODiff diff(last_mobile_state, mobile_state);

    // When B1 is pressed, it should:
    // - get last feedback
    // - save the joint positions as positions a waypoint



    // Buttton B1 - Home Position
    if (diff.get(1) == MobileIODiff::ButtonState::ToOn)
    {
      // positions << 0, 0, 0, 0, 0, 0;
      // single_time = 3;
      addWaypoint(state, arm -> lastFeedback());

      // arm -> setGoal(arm::Goal(state._waypoints[0]._positions));
    }

    if (diff.get(2) == MobileIODiff::ButtonState::ToOn)
    {
      
      const arm::Goal bob = playWaypoints(state);
      printf("It's all good till here\n");
      arm -> setGoal(bob);
      // playWaypoints(state);
    }














    // // Buttton B1 - Home Position
    // if (diff.get(1) == MobileIODiff::ButtonState::ToOn)
    // {
    //   positions << 0, 0, 0, 0, 0, 0;
    //   single_time = 3;
    //   arm -> setGoal(arm::Goal(single_time, positions));
    // }

    // // Button B2 - Waypoint 1
    // if (diff.get(2) == MobileIODiff::ButtonState::ToOn)
    // {
    //   positions << M_PI/4, M_PI/3, 2*M_PI/3, M_PI/3, M_PI/4, 0;
    //   single_time = 3;
    //   arm -> setGoal(arm::Goal(single_time, positions));
    // }

    // // Button B5 - Waypoint 2
    // if (diff.get(5) == MobileIODiff::ButtonState::ToOn)
    // {
    //   positions << -M_PI/4, M_PI/3, 2*M_PI/3, M_PI/3, 3*M_PI/4, 0;
    //   single_time = 3;
    //   arm -> setGoal(arm::Goal(single_time, positions));
    // }

    // // Button B6 - Grav Comp Mode
    // if (diff.get(6) == MobileIODiff::ButtonState::ToOn)
    // {
    //   // cancel any goal that is set, returning arm into gravComp mode
    //   arm -> cancelGoal();
    // }

    // // Button B8 - End Demo
    // if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn)
    // {
    //   // Clear MobileIO text
    //   mobile -> clearText();
    //   return 1;
    // }

    // Update to the new last_state
    last_mobile_state = mobile_state;

    // Send latest commands to the arm
    arm->send();
  }

  // Clear MobileIO text
  mobile -> clearText();

  return 0;
}



