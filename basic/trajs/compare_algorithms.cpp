/**
 * Generate a trajectory and execute it.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * September 2018
 */

#include "trajectory.hpp"
#include "Eigen/Eigen"

#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "command.hpp"
#include <math.h>
#include <chrono>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "util/plot_functions.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace plt = matplotlibcpp;

/**
 * Execute a trajectory on physical hardware.
 * 
 * @param trajectory The trajectory to execute
 * @param trajectoryName The name of the trajectory to display
 * @return true if successful, false otherwise
 */
bool execute(const std::shared_ptr<hebi::trajectory::Trajectory>& trajectory, const std::string& trajectoryName) {
  // Validate trajectory duration - don't attempt to execute unreasonably long trajectories
  double duration = trajectory->getDuration();
  if (duration > 100.0) {
    std::cout << "Skipping execution of " << trajectoryName << " trajectory - duration (" 
              << duration << " seconds) is too long." << std::endl;
    return false;
  }

  // Get group
  hebi::Lookup lookup;
  auto group = lookup.getGroupFromNames({"Arm"}, {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1"});

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return false;
  }

  // Get the current feedback from the module; this allows us to start at the
  // current position
  int num_joints = group->size();
  hebi::GroupFeedback fbk(num_joints);
  
  if (!group->getNextFeedback(fbk)) {
    std::cout << "Error getting feedback." << std::endl;
    return false;
  }

  // Start logging in the background
  std::cout << "Executing " << trajectoryName << " trajectory on hardware..." << std::endl;
  std::string log_path = group->startLog("../logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return false;
  }

  // Follow the trajectory
  hebi::GroupCommand cmd(num_joints);
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);

  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);

  std::cout << "Following trajectory for " << duration << " seconds..." << std::endl;
  
  while (t.count() < duration) {
    // "getNextFeedback" serves to rate limit the loop without calling sleep
    group->getNextFeedback(fbk);
    t = std::chrono::system_clock::now() - start;

    // Pass "nullptr" in to ignore a term. Position and time are always returned.
    trajectory->getState(t.count(), &pos_cmd, &vel_cmd, nullptr);
    cmd.setPosition(pos_cmd);
    cmd.setVelocity(vel_cmd);
    group->sendCommand(cmd);
  }

  // Stop logging
  auto log_file = group->stopLog();
  std::cout << "Trajectory execution complete. Log saved to: " << log_path << std::endl;
  
  return true;
}

int main(int argc, char **argv)
{
  /*----------- PARSE ARGUMENTS ------------*/
  // Default parameters
  int num_joints = 1;
  int num_waypoints = 10;
  bool show_plot = true;
  int custom_seed = -1;
  std::string waypoints_file = ""; // Path to waypoints file (if provided)
  bool execute_enabled = true;   // Whether to execute trajectories on hardware (default: true)

  // Process command line arguments
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    if (arg == "--no-plot")
    {
      show_plot = false;
    }
    else if (arg == "--joints" && i + 1 < argc)
    {
      num_joints = std::stoi(argv[++i]);
      if (num_joints <= 0)
      {
        std::cerr << "Number of joints must be positive" << std::endl;
        return 1;
      }
    }
    else if (arg == "--waypoints" && i + 1 < argc)
    {
      num_waypoints = std::stoi(argv[++i]);
      if (num_waypoints < 2)
      {
        std::cerr << "Number of waypoints must be at least 2" << std::endl;
        return 1;
      }
    }
    else if (arg == "--seed" && i + 1 < argc)
    {
      custom_seed = std::stoi(argv[++i]);
    }
    else if (arg == "--waypoints-file" && i + 1 < argc)
    {
      waypoints_file = argv[++i];
    }
    else if (arg == "--no-execute")
    {
      execute_enabled = false;
    }
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
      std::cout << "Options:" << std::endl;
      std::cout << "  --no-plot               Disable plotting" << std::endl;
      std::cout << "  --joints <number>       Set number of joints (default: 10)" << std::endl;
      std::cout << "  --waypoints <number>    Set number of waypoints (default: 10)" << std::endl;
      std::cout << "  --seed <number>         Set random seed for waypoint generation (default: random)" << std::endl;
      std::cout << "  --waypoints-file <path> Path to a CSV/TXT file with waypoints" << std::endl;
      std::cout << "  --no-execute            Disable trajectory execution on hardware" << std::endl;
      std::cout << "  --help, -h              Show this help message" << std::endl;
      return 0;
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << std::endl;
      std::cerr << "Use --help for usage information" << std::endl;
      return 1;
    }
  }

  std::cout << "Running with " << num_joints << " joints, "
            << (waypoints_file.empty() ? std::to_string(num_waypoints) + " waypoints" : "waypoints from " + waypoints_file)
            << ", plotting " << (show_plot ? "enabled" : "disabled") << std::endl;

  /*----------- COMMON ------------*/
  // Define trajectory parameters

  // Create waypoint matrices for both trajectory generators
  Eigen::MatrixXd positions;
  Eigen::MatrixXd velocities;
  Eigen::MatrixXd accelerations;

  if (!waypoints_file.empty()) {
    // Read waypoints from file
    std::ifstream file(waypoints_file);
    if (!file.is_open()) {
      std::cerr << "Error: Unable to open waypoints file: " << waypoints_file << std::endl;
      return 1;
    }

    std::vector<std::vector<double>> waypoint_data;
    std::string line;

    // Read the file line by line
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::vector<double> row;
      std::string value;
      
      // Parse values separated by comma or space
      while (std::getline(iss, value, ',')) {
        // Remove any leading/trailing whitespace
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        
        if (!value.empty()) {
          try {
            double val = std::stod(value);
            row.push_back(val);
          }
          catch (const std::invalid_argument&) {
            continue; // Skip non-numeric values
          }
        }
      }
      
      if (!row.empty()) {
        waypoint_data.push_back(row);
      }
    }

    if (waypoint_data.empty()) {
      std::cerr << "Error: No valid waypoints found in file" << std::endl;
      return 1;
    }

    // Determine the number of joints and waypoints
    size_t file_num_waypoints = waypoint_data.size();
    size_t file_num_joints = waypoint_data[0].size();
    
    // Check if all rows have the same number of columns
    for (size_t i = 1; i < waypoint_data.size(); i++) {
      if (waypoint_data[i].size() != file_num_joints) {
        std::cerr << "Error: Inconsistent number of joint positions in waypoints file at row " << (i+1) << std::endl;
        return 1;
      }
    }

    // Update the dimensions
    num_joints = file_num_joints;
    num_waypoints = file_num_waypoints;

    // Initialize matrices with the right dimensions
    positions = Eigen::MatrixXd(num_joints, num_waypoints);
    velocities = Eigen::MatrixXd(num_joints, num_waypoints);
    accelerations = Eigen::MatrixXd(num_joints, num_waypoints);

    // Fill the positions matrix with data from file
    for (int i = 0; i < num_waypoints; i++) {
      for (int j = 0; j < num_joints; j++) {
        positions(j, i) = waypoint_data[i][j];
      }
      // Set velocities and accelerations to NaN (unconstrained) except for start/end
      velocities.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
      accelerations.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
    }

    // Set zero velocity and acceleration constraints for start and end points
    velocities.col(0) = Eigen::VectorXd::Zero(num_joints);
    accelerations.col(0) = Eigen::VectorXd::Zero(num_joints);
    velocities.col(num_waypoints - 1) = Eigen::VectorXd::Zero(num_joints);
    accelerations.col(num_waypoints - 1) = Eigen::VectorXd::Zero(num_joints);

    std::cout << "Successfully loaded " << num_waypoints << " waypoints for " << num_joints << " joints from file." << std::endl;
  } 
  else {
    // Generate random waypoints as before
    positions = Eigen::MatrixXd(num_joints, num_waypoints);
    velocities = Eigen::MatrixXd(num_joints, num_waypoints);
    accelerations = Eigen::MatrixXd(num_joints, num_waypoints);

    // Start point with full constraints
    positions.col(0) = Eigen::VectorXd::Zero(num_joints);

    // Generate random waypoints between 0 and M_PI for intermediate points
    int seed = (custom_seed != -1) ? custom_seed : std::chrono::system_clock::now().time_since_epoch().count();
    std::cout << "Random seed is " << seed << std::endl;
    srand(seed); // Seed the random number generator
    for (int i = 1; i < num_waypoints - 1; i++)
    {
      for (int j = 0; j < num_joints; j++)
      {
        positions(j, i) = ((double)rand() / RAND_MAX) * M_PI;
      }
      velocities.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
      accelerations.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
    }

    // End point with position and velocity constraints
    positions.col(num_waypoints - 1) = Eigen::VectorXd::Constant(num_joints, M_PI_2);
    velocities.col(num_waypoints - 1) = Eigen::VectorXd::Zero(num_joints);
    accelerations.col(num_waypoints - 1) = Eigen::VectorXd::Zero(num_joints);
  }

  // Set maximum velocity and acceleration
  Eigen::VectorXd max_v = Eigen::VectorXd::Constant(num_joints, 5.0);    // m/s
  Eigen::VectorXd max_a = Eigen::VectorXd::Constant(num_joints, 10.0); // m/s^2

  Eigen::VectorXd times;
  hebi::trajectory::Trajectory::estimateBestTimes(positions, max_v, max_a, times);

  Eigen::VectorXd segment_times;
  hebi::trajectory::Trajectory::timeVectorToSegmentTimes(times, segment_times);

  // Print the estimated segment times
  std::cout << "Estimated segment times: ";
  for (int i = 0; i < segment_times.size(); i++)
  {
    segment_times(i) *= 0.5;
    std::cout << segment_times(i) << " ";
  }
  std::cout << std::endl;

  hebi::trajectory::Trajectory::segmentTimestoTimeVector(segment_times, times);

  // Vector to hold all trajectories and their properties
  std::vector<std::shared_ptr<hebi::trajectory::Trajectory>> trajectories;
  std::vector<std::string> traj_names;
  std::vector<std::chrono::duration<double>> elapsed_times;
  std::vector<Eigen::VectorXd> time_vectors;
  
  // Current API trajectory with heuristic time estimation
  auto start_time = std::chrono::high_resolution_clock::now();
  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
  auto end_time = std::chrono::high_resolution_clock::now();
  trajectories.push_back(trajectory);
  traj_names.push_back("Basic");
  elapsed_times.push_back(end_time - start_time);
  time_vectors.push_back(times);

  std::cout << "Basic trajectory created successfully." << std::endl;
  
  start_time = std::chrono::high_resolution_clock::now();
  auto trajectory2 = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
  trajectory2->scaleTrajectoryToMeetLimits(max_v, max_a);
  end_time = std::chrono::high_resolution_clock::now();
  trajectories.push_back(trajectory2);
  traj_names.push_back("Scaled");
  elapsed_times.push_back(end_time - start_time);
  time_vectors.push_back(trajectory2->getTimeVector());
  
  // Check if the scaled trajectory has the same duration as the original one
  bool trajectory_within_limits = std::abs(trajectory->getDuration() - trajectory2->getDuration()) < 1e-6;
  if (trajectory_within_limits) {
    std::cout << "Scaled trajectory has the same duration as Basic - original trajectory was already within limits!" << std::endl;
  } else {
    std::cout << "Scaled trajectory created successfully." << std::endl;
  }
  
  // Minimize time using optimizer (Richter)
  hebi::trajectory::NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 1000;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 500000.0;
  parameters.use_soft_constraints = true;
  parameters.soft_constraint_weight = 1.5;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  parameters.equality_constraint_tolerance = 1.0e-3;
  parameters.print_debug_info_time_allocation = false;
  parameters.print_debug_info = false;
  parameters.minimum_segment_time = 0.01;
  parameters.time_alloc_method = hebi::trajectory::NonlinearOptimizationParameters::kRichterTime;
  parameters.algorithm = nlopt::LN_BOBYQA;
  if (parameters.time_alloc_method == hebi::trajectory::NonlinearOptimizationParameters::kMellingerOuterLoop)
  {
    parameters.algorithm = nlopt::LD_LBFGS;
  }
  
  start_time = std::chrono::high_resolution_clock::now();
  auto trajectory3 = hebi::trajectory::Trajectory::createMinTimeTrajectory(parameters, positions, &velocities, &accelerations, &max_v, &max_a);
  trajectory3->scaleTrajectoryToMeetLimits(max_v, max_a);
  end_time = std::chrono::high_resolution_clock::now();
  trajectories.push_back(trajectory3);
  traj_names.push_back("Richter");
  elapsed_times.push_back(end_time - start_time);
  time_vectors.push_back(trajectory3->getTimeVector());

  std::cout << "Richter trajectory created successfully." << std::endl;

  // Create a trajectory with a different time allocation method (Mellinger)
  hebi::trajectory::NonlinearOptimizationParameters parameters4;
  parameters4.max_iterations = 1000;
  parameters4.f_rel = 0.05;
  parameters4.x_rel = 0.1;
  parameters4.time_penalty = 5000.0;
  parameters4.use_soft_constraints = true;
  parameters4.soft_constraint_weight = 1.5;
  parameters4.initial_stepsize_rel = 0.1;
  parameters4.inequality_constraint_tolerance = 0.1;
  parameters4.equality_constraint_tolerance = 1.0e-3;
  parameters4.print_debug_info_time_allocation = false;
  parameters4.print_debug_info = false;
  parameters4.minimum_segment_time = 0.01;
  parameters4.time_alloc_method = hebi::trajectory::NonlinearOptimizationParameters::kMellingerOuterLoop;
  parameters4.algorithm = nlopt::LD_LBFGS;
  
  start_time = std::chrono::high_resolution_clock::now();
  auto trajectory4 = hebi::trajectory::Trajectory::createMinTimeTrajectory(parameters4, positions, &velocities, &accelerations, &max_v, &max_a);
  trajectory4->scaleTrajectoryToMeetLimits(max_v, max_a);
  end_time = std::chrono::high_resolution_clock::now();
  trajectories.push_back(trajectory4);
  traj_names.push_back("Mellinger");
  elapsed_times.push_back(end_time - start_time);
  time_vectors.push_back(trajectory4->getTimeVector());

  std::cout << "Mellinger trajectory created successfully." << std::endl;

  /*--------------------------------------------------------------------------------------------*/

  // Print time measurements
  std::cout << "--------- Trajectory Generation Times ---------" << std::endl;
  for (size_t i = 0; i < trajectories.size(); i++) {
    std::cout << traj_names[i] << " Trajectory: " << elapsed_times[i].count() << " seconds" << std::endl;
  }
  std::cout << "---------------------------------------------" << std::endl;

  // Determine which trajectories to show and execute based on duration
  std::vector<bool> show_trajs;
  for (size_t i = 0; i < trajectories.size(); i++) {
    double duration = trajectories[i]->getDuration();
    bool show_traj = true;
    
    // Skip the Scaled trajectory if it has the same duration as Basic
    if (i == 1 && trajectory_within_limits) {
      show_traj = false;
      std::cout << "Skipping Scaled trajectory display/execution since it's identical to Basic trajectory" << std::endl;
    } 
    // Skip trajectories that are too long
    else if (duration > 100.0) {
      show_traj = false;
      std::cout << "Trajectory " << traj_names[i] << " duration: " << duration << " seconds (too long, will skip plotting and execution)" << std::endl;
    } else {
      std::cout << "Trajectory " << traj_names[i] << " duration: " << duration << " seconds" << std::endl;
    }
    
    show_trajs.push_back(show_traj);
  }

  // Check if any trajectories are plottable/executable
  if (std::none_of(show_trajs.begin(), show_trajs.end(), [](bool v) { return v; })) {
    std::cout << "All trajectories are too long. Skipping plotting and execution." << std::endl;
    return 0;
  }

  if (show_plot) {
  try
  {
    std::cout << "Generating trajectory plots..." << std::endl;
    
    // Get trajectory durations
    std::vector<double> durations;
    
    // Calculate durations
    for (const auto& trajectory : trajectories) {
      double duration = trajectory->getDuration();
      durations.push_back(duration);
    }

    // Generate time points for plotting
    std::vector<std::vector<double>> time_points;
    for (size_t i = 0; i < trajectories.size(); i++) {
      if (show_trajs[i]) {
        time_points.push_back(linspace(0.0, durations[i], 1000.0));
      } else {
        time_points.push_back(std::vector<double>());
      }
    }

    // Colors for each trajectory
    std::vector<std::string> colors = {"-b", "-r", "-g", "-m", "-c", "-y"};
    while (colors.size() < trajectories.size()) {
      colors.push_back(colors[colors.size() % colors.size()]);
    }

    for (int i = 0; i < num_joints; i++)
    {
      // Create vectors to hold trajectory data
      std::vector<std::vector<double>> pos_data(trajectories.size());
      std::vector<std::vector<double>> vel_data(trajectories.size());
      std::vector<std::vector<double>> acc_data(trajectories.size());
      
      // Get trajectory data for each trajectory
      for (size_t t = 0; t < trajectories.size(); t++) {
        if (show_trajs[t]) {
          // Position data
          pos_data[t] = f_x(time_points[t], [&, i, t](double time) {
            Eigen::VectorXd pos(num_joints); 
            trajectories[t]->getState(time, &pos, nullptr, nullptr); 
            return pos[i];
          });
          
          // Velocity data
          vel_data[t] = f_x(time_points[t], [&, i, t](double time) {
            Eigen::VectorXd vel(num_joints); 
            trajectories[t]->getState(time, nullptr, &vel, nullptr); 
            return vel[i];
          });
          
          // Acceleration data
          acc_data[t] = f_x(time_points[t], [&, i, t](double time) {
            Eigen::VectorXd acc(num_joints); 
            trajectories[t]->getState(time, nullptr, nullptr, &acc); 
            return acc[i];
          });
        }
      }

      // Create subplots for position, velocity, and acceleration
      plt::figure();

      // Position subplot
      plt::subplot(3, 1, 1);
      
      // Prepare waypoint positions once
      std::vector<double> waypoint_positions;
      for (int j = 0; j < num_waypoints; j++) {
        waypoint_positions.push_back(positions(i, j));
      }
      
      // Plot each trajectory
      for (size_t t = 0; t < trajectories.size(); t++) {
        if (show_trajs[t]) {
          plt::named_plot(traj_names[t], time_points[t], pos_data[t], colors[t]);
          
          // Plot waypoints for each trajectory
          if (!time_vectors[t].size()) continue;
          std::vector<double> waypoint_times(time_vectors[t].data(), time_vectors[t].data() + time_vectors[t].size());
          plt::named_plot("Waypoints (" + traj_names[t] + ")", waypoint_times, waypoint_positions, colors[t].substr(1, 2) + "o");
        }
      }

      plt::title("Position Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Position (rad)");
      plt::legend();

      // Velocity subplot
      plt::subplot(3, 1, 2);
      
      // Plot each trajectory's velocity
      for (size_t t = 0; t < trajectories.size(); t++) {
        if (show_trajs[t]) {
          plt::named_plot(traj_names[t], time_points[t], vel_data[t], colors[t]);
        }
      }

      // Add velocity limits - use first trajectory's time points as reference
      size_t ref_idx = 0;
      while (ref_idx < time_points.size() && time_points[ref_idx].empty()) ref_idx++;
      
      if (ref_idx < time_points.size() && max_v[i] < 1E3) {
        std::vector<double> max_vel_limit(time_points[ref_idx].size(), max_v[i]);
        std::vector<double> min_vel_limit(time_points[ref_idx].size(), -max_v[i]);
        plt::named_plot("Max Velocity Limit", time_points[ref_idx], max_vel_limit, "--k");
        plt::named_plot("Min Velocity Limit", time_points[ref_idx], min_vel_limit, "--k");
      }

      plt::title("Velocity Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Velocity (rad/s)");
      plt::legend();

      // Acceleration subplot
      plt::subplot(3, 1, 3);
      
      // Plot each trajectory's acceleration
      for (size_t t = 0; t < trajectories.size(); t++) {
        if (show_trajs[t]) {
          plt::named_plot(traj_names[t], time_points[t], acc_data[t], colors[t]);
        }
      }

      if (ref_idx < time_points.size() && max_a[i] < 1E3) {
        std::vector<double> max_acc_limit(time_points[ref_idx].size(), max_a[i]);
        std::vector<double> min_acc_limit(time_points[ref_idx].size(), -max_a[i]);
        plt::named_plot("Max Acceleration Limit", time_points[ref_idx], max_acc_limit, "--k");
        plt::named_plot("Min Acceleration Limit", time_points[ref_idx], min_acc_limit, "--k");
      }

      plt::title("Acceleration Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Acceleration (rad/s²)");
      plt::legend();

      plt::show();
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "Note: plotting failed - " << e.what() << std::endl;
  }
  }

  // Execute the trajectories on hardware
  if (execute_enabled) {
    std::cout << "\n--------- Executing Trajectories on Hardware ---------" << std::endl;
    for (size_t i = 0; i < trajectories.size(); i++) {
      if (show_trajs[i] && !execute(trajectories[i], traj_names[i])) {
        std::cerr << "Error executing trajectory: " << traj_names[i] << std::endl;
      }
    }
    std::cout << "---------------------------------------------------" << std::endl;
  } else {
    std::cout << "\nSkipping trajectory execution on hardware (--no-execute)" << std::endl;
  }

  return 0;
}
