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
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include "command.hpp"
#include <math.h>
#include <chrono>
#include <vector>
#include <iostream>
#include "util/plot_functions.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace plt = matplotlibcpp;

// Function to plan a trajectory using MAV trajectory generation
bool planMavTrajectory(
    const mav_trajectory_generation::Vertex::Vector &vertices,
    const std::vector<double> &v_max,
    const std::vector<double> &a_max,
    const std::vector<double> &initial_segment_times,
    mav_trajectory_generation::Trajectory *trajectory)
{
  if (vertices.empty()) {
    std::cerr << "No waypoints provided!" << std::endl;
    return false;
  }
  
  const int num_joints = vertices[0].D(); // Number of joints (dimensions) in the trajectory
  
  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 1000;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 5000.0;
  parameters.use_soft_constraints = true;
  parameters.soft_constraint_weight = 1.5;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  parameters.equality_constraint_tolerance = 1.0e-3;
  parameters.algorithm = nlopt::LN_BOBYQA;
  parameters.print_debug_info_time_allocation = true;
  parameters.print_debug_info = true;
  parameters.segment_wise = false;
  parameters.minimum_segment_time = 0.01;
  parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTime;

  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;

  // Use provided segment times
  std::vector<double> segment_times = initial_segment_times;

  // Ensure segment_times are greater than or equal to the minimum segment time
  for (double &time : segment_times) {
    if (time < parameters.minimum_segment_time) {
      time = parameters.minimum_segment_time;
    }
  }

  // set up optimization problem
  const int N = 6;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(num_joints, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration if specified
  for (int i = 0; i < num_joints; ++i) {
    opt.addMaximumMagnitudeConstraint(i, mav_trajectory_generation::derivative_order::VELOCITY, v_max[i]);
    opt.addMaximumMagnitudeConstraint(i, mav_trajectory_generation::derivative_order::ACCELERATION, a_max[i]);
  }

  // solve trajectory
  opt.optimize();

  if (opt.getOptimizationInfo().stopping_reason >= 1 && opt.getOptimizationInfo().stopping_reason != 6) {
    opt.getTrajectory(trajectory);
    return true;
  } else {
    std::cerr << "[MAV]: Trajectory optimization failed!" << std::endl;
    return false;
  }
}

int main(int argc, char **argv)
{
  /*----------- PARSE ARGUMENTS ------------*/
  // Default parameters
  int num_joints = 1;
  int num_waypoints = 10;
  bool show_plot = true;

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
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
      std::cout << "Options:" << std::endl;
      std::cout << "  --no-plot               Disable plotting" << std::endl;
      std::cout << "  --joints <number>       Set number of joints (default: 10)" << std::endl;
      std::cout << "  --waypoints <number>    Set number of waypoints (default: 10)" << std::endl;
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
            << num_waypoints << " waypoints, plotting "
            << (!show_plot ? "enabled" : "disabled") << std::endl;

  /*----------- COMMON ------------*/
  // Define trajectory parameters

  // Create waypoint matrices for both trajectory generators
  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // Start point with full constraints
  positions.col(0) = Eigen::VectorXd::Zero(num_joints);

  // Generate random waypoints between 0 and M_PI for intermediate points
  // int seed = std::chrono::system_clock::now().time_since_epoch().count();
  int seed = 264196044; // Fixed seed for reproducibility
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

  // Set maximum velocity and acceleration
  Eigen::VectorXd max_v = Eigen::VectorXd::Constant(num_joints, 5.0);    // m/s
  Eigen::VectorXd max_a = Eigen::VectorXd::Constant(num_joints, 1000.0); // m/s^2

  Eigen::VectorXd times;
  hebi::trajectory::Trajectory::estimateBestTimes(positions, max_v, max_a, times);
  // times[1] = 0.1; // Set the first time to 0.1 seconds

  // Current API trajectory with heuristic time estimation
  auto start_time1 = std::chrono::high_resolution_clock::now();
  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
  auto end_time1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time1 = end_time1 - start_time1;
  
  // Scaling the whole trajectory to meet limits
  auto start_time2 = std::chrono::high_resolution_clock::now();
  auto trajectory2 = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
  trajectory2->scaleTrajectoryToMeetLimits(max_v, max_a);
  auto end_time2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time2 = end_time2 - start_time2;
  
  auto time_vector2 = trajectory2->getTimeVector();
  
  // Minimize time using optimizer
  hebi::trajectory::NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 50;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 100.0;
  parameters.use_soft_constraints = false;
  parameters.soft_constraint_weight = 1.5;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  parameters.equality_constraint_tolerance = 1.0e-3;
  parameters.algorithm = nlopt::LD_LBFGS;
  parameters.print_debug_info_time_allocation = true;
  parameters.print_debug_info = true;
  parameters.minimum_segment_time = 0.01;
  
  auto start_time3 = std::chrono::high_resolution_clock::now();
  auto trajectory3 = hebi::trajectory::Trajectory::createMinTimeTrajectory(parameters, positions, &velocities, &accelerations, &max_v, &max_a);
  trajectory3->scaleTrajectoryToMeetLimits(max_v, max_a);
  auto end_time3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time3 = end_time3 - start_time3;
  
  auto time_vector3 = trajectory3->getTimeVector();

  // MAV trajectory generation
  mav_trajectory_generation::Vertex::Vector vertices;
  for (int i = 0; i < num_waypoints; i++) {
    mav_trajectory_generation::Vertex vertex(num_joints);
    vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, positions.col(i));
    if (i == 0 || i == num_waypoints - 1) {
      vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, velocities.col(i));
      vertex.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, accelerations.col(i));
    }
    vertices.push_back(vertex);
  }

  std::vector<double> v_max(num_joints, 5.0);
  std::vector<double> a_max(num_joints, 1000.0);
  Eigen::VectorXd segment_times;
  hebi::trajectory::Trajectory::timeVectorToSegmentTimes(times, segment_times);
  std::vector<double> initial_segment_times(segment_times.data(), segment_times.data() + segment_times.size());

  auto start_time4 = std::chrono::high_resolution_clock::now();
  mav_trajectory_generation::Trajectory trajectory4;
  bool success = planMavTrajectory(vertices, v_max, a_max, initial_segment_times, &trajectory4);
  auto end_time4 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time4 = end_time4 - start_time4;

  auto time_vector4 = trajectory4.getSegmentTimes();

  // Print time measurements
  std::cout << "--------- Trajectory Generation Times ---------" << std::endl;
  std::cout << "Basic Trajectory:       " << elapsed_time1.count() << " seconds" << std::endl;
  std::cout << "Scaled Trajectory:      " << elapsed_time2.count() << " seconds" << std::endl;
  std::cout << "Time-Optimized:         " << elapsed_time3.count() << " seconds" << std::endl;
  std::cout << "MAV Trajectory:         " << elapsed_time4.count() << " seconds" << std::endl;
  std::cout << "---------------------------------------------" << std::endl;

  if (!show_plot)
  {
    return 0;
  }
  try
  {
    std::cout << "Generating trajectory plots..." << std::endl;
    
    // Get trajectory duration
    double duration1 = trajectory->getDuration();
    double duration2 = trajectory2->getDuration();
    double duration3 = trajectory3->getDuration();
    double duration4 = trajectory4.getMaxTime();
    // Print trajectory durations
    std::cout << "Trajectory 1 duration: " << duration1 << " seconds" << std::endl;
    std::cout << "Trajectory 2 duration: " << duration2 << " seconds" << std::endl;
    std::cout << "Trajectory 3 duration: " << duration3 << " seconds" << std::endl;
    std::cout << "Trajectory 4 duration: " << duration4 << " seconds" << std::endl;

    bool show_traj1 = true;
    bool show_traj2 = true;
    bool show_traj3 = true;
    bool show_traj4 = true;

    // If duration is too long, disable plotting
    if (duration1 > 1E3)
    {
      std::cout << "Trajectory 1 duration is too long, disabling plotting." << std::endl;
      show_traj1 = false;
    }
    if (duration2 > 1E3)
    {
      std::cout << "Trajectory 2 duration is too long, disabling plotting." << std::endl;
      show_traj2 = false;
    }
    if (duration3 > 1E3)
    {
      std::cout << "Trajectory 3 duration is too long, disabling plotting." << std::endl;
      show_traj3 = false;
    }
    if (duration4 > 1E3)
    {
      std::cout << "Trajectory 4 duration is too long, disabling plotting." << std::endl;
      show_traj4 = false;
    }
    if (!success)
    {
      std::cout << "MAV trajectory generation failed, disabling plotting." << std::endl;
      show_traj4 = false;
    }

    if (!show_traj1 && !show_traj2 && !show_traj3 && !show_traj4)
    {
      std::cout << "All trajectories are too long, disabling plotting." << std::endl;
      return 0;
    }

    // Generate time points for plotting
    std::vector<double> time_points1, time_points2, time_points3, time_points4;
    if (show_traj1)
      time_points1 = linspace(0.0, duration1, 1000.0);
    if (show_traj2)
      time_points2 = linspace(0.0, duration2, 1000.0);
    if (show_traj3)
      time_points3 = linspace(0.0, duration3, 1000.0);
    if (show_traj4)
      time_points4 = linspace(0.0, duration4, 1000.0);

    for (int i = 0; i < num_joints; i++)
    {
      // Get trajectory data for the first trajectory
      std::vector<double> pos_data1, vel_data1, acc_data1;
      if (show_traj1)
      {
        pos_data1 = f_x(time_points1, [&, i](double t)
                        {
          Eigen::VectorXd pos(num_joints); 
          trajectory->getState(t, &pos, nullptr, nullptr); 
          return pos[i]; });

        vel_data1 = f_x(time_points1, [&, i](double t)
                        {
          Eigen::VectorXd vel(num_joints); 
          trajectory->getState(t, nullptr, &vel, nullptr); 
          return vel[i]; });

        acc_data1 = f_x(time_points1, [&, i](double t)
                        {
          Eigen::VectorXd acc(num_joints); 
          trajectory->getState(t, nullptr, nullptr, &acc); 
          return acc[i]; });
      }

      std::vector<double> pos_data2, vel_data2, acc_data2;
      if (show_traj2)
      {
        pos_data2 = f_x(time_points2, [&, i](double t)
                        {
          Eigen::VectorXd pos(num_joints); 
          trajectory2->getState(t, &pos, nullptr, nullptr); 
          return pos[i]; });

        vel_data2 = f_x(time_points2, [&, i](double t)
                        {
          Eigen::VectorXd vel(num_joints); 
          trajectory2->getState(t, nullptr, &vel, nullptr); 
          return vel[i]; });

        acc_data2 = f_x(time_points2, [&, i](double t)
                        {
          Eigen::VectorXd acc(num_joints); 
          trajectory2->getState(t, nullptr, nullptr, &acc); 
          return acc[i]; });
      }

      std::vector<double> pos_data3, vel_data3, acc_data3;
      if (show_traj3)
      {
        pos_data3 = f_x(time_points3, [&, i](double t)
                        {
          Eigen::VectorXd pos(num_joints); 
          trajectory3->getState(t, &pos, nullptr, nullptr); 
          return pos[i]; });

        vel_data3 = f_x(time_points3, [&, i](double t)
                        {
          Eigen::VectorXd vel(num_joints); 
          trajectory3->getState(t, nullptr, &vel, nullptr); 
          return vel[i]; });

        acc_data3 = f_x(time_points3, [&, i](double t)
                        {
          Eigen::VectorXd acc(num_joints); 
          trajectory3->getState(t, nullptr, nullptr, &acc); 
          return acc[i]; });
      }

      std::vector<double> pos_data4, vel_data4, acc_data4;
      if (show_traj4)
      {
        pos_data4 = f_x(time_points4, [&, i](double t)
                        {
          Eigen::VectorXd point = trajectory4.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
          return point[i]; });

        vel_data4 = f_x(time_points4, [&, i](double t)
                        {
          Eigen::VectorXd point = trajectory4.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY); 
          return point[i]; });

        acc_data4 = f_x(time_points4, [&, i](double t)
                        {
          Eigen::VectorXd point = trajectory4.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);
          return point[i]; });
      }

      // Create subplots for position, velocity, and acceleration
      plt::figure();

      // Position subplot
      plt::subplot(3, 1, 1);
      if (show_traj1)
        plt::named_plot("Basic", time_points1, pos_data1, "-b");
      if (show_traj2)
        plt::named_plot("Scaled", time_points2, pos_data2, "-r");
      if (show_traj3)
        plt::named_plot("Time-Optimized", time_points3, pos_data3, "-g");
      if (show_traj4)
        plt::named_plot("MAV", time_points4, pos_data4, "-m");

      // Plot position waypoints
      std::vector<double> waypoint_times1, waypoint_positions1;
      for (int j = 0; j < num_waypoints; j++)
      {
        waypoint_times1.push_back(times(j));
        waypoint_positions1.push_back(positions(i, j));
      }
      if (show_traj1) {
        plt::named_plot("Waypoints (Basic)", waypoint_times1, waypoint_positions1, "bo");
      }

      if (show_traj2)
      {
        std::vector<double> waypoint_times2(time_vector2.data(), time_vector2.data() + time_vector2.size());
        plt::named_plot("Waypoints (Scaled)", waypoint_times2, waypoint_positions1, "ro");
      }

      if (show_traj3)
      {
        std::vector<double> waypoint_times3(time_vector3.data(), time_vector3.data() + time_vector3.size());
        plt::named_plot("Waypoints (Time-Optimized)", waypoint_times3, waypoint_positions1, "go");
      }

      // if (show_traj4)
      // {
      //   plt::named_plot("Waypoints (MAV)", time_vector4, waypoint_positions1, "mo");
      // }

      plt::title("Position Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Position (rad)");
      plt::legend();

      // Velocity subplot
      plt::subplot(3, 1, 2);
      if (show_traj1)
        plt::named_plot("Basic", time_points1, vel_data1, "-b");
      if (show_traj2)
        plt::named_plot("Scaled", time_points2, vel_data2, "-r");
      if (show_traj3)
        plt::named_plot("Time-Optimized", time_points3, vel_data3, "-g");
      if (show_traj4)
        plt::named_plot("MAV", time_points4, vel_data4, "-m");

      // Add velocity limits
      std::vector<double> max_vel_limit(time_points1.size(), max_v[i]);
      std::vector<double> min_vel_limit(time_points1.size(), -max_v[i]);
      plt::named_plot("Max Velocity Limit", time_points1, max_vel_limit, "--k");
      plt::named_plot("Min Velocity Limit", time_points1, min_vel_limit, "--k");

      plt::title("Velocity Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Velocity (rad/s)");
      plt::legend();

      // Acceleration subplot
      plt::subplot(3, 1, 3);
      if (show_traj1)
        plt::named_plot("Basic", time_points1, acc_data1, "-b");
      if (show_traj2)
        plt::named_plot("Scaled", time_points2, acc_data2, "-r");
      if (show_traj3)
        plt::named_plot("Time-Optimized", time_points3, acc_data3, "-g");
      if (show_traj4)
        plt::named_plot("MAV", time_points4, acc_data4, "-m");

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

  return 0;
}
