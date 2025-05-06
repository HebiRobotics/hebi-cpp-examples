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
  srand(7); // Seed the random number generator
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

  // Generate trajectory
  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);

  // Get trajectory duration
  double duration = trajectory->getDuration();
  std::cout << "Trajectory duration: " << duration << " seconds" << std::endl;
  
  Eigen::MatrixXd pos_minmax_candidates(num_joints, num_waypoints - 1);
  Eigen::MatrixXd vel_minmax_candidates(num_joints, num_waypoints - 1);
  Eigen::MatrixXd acc_minmax_candidates(num_joints, num_waypoints - 1);

  pos_minmax_candidates.setZero();
  vel_minmax_candidates.setZero();
  acc_minmax_candidates.setZero();

  for (int j = 0; j < num_waypoints - 1; j++)
  {
    Eigen::VectorXd p_max(num_joints), v_max(num_joints), a_max(num_joints);
    trajectory->getMaxInSegment(j, 0, p_max);
    trajectory->getMaxInSegment(j, 1, v_max);
    trajectory->getMaxInSegment(j, 2, a_max);
    pos_minmax_candidates.col(j) = p_max;
    vel_minmax_candidates.col(j) = v_max;
    acc_minmax_candidates.col(j) = a_max;
  }

  if (!show_plot)
  {
    return 0;
  }
  try
  {
    std::cout << "Generating trajectory plots..." << std::endl;

    // Generate time points for plotting
    auto time_points = linspace(0.0, duration, 1000.0);

    for (int i = 0; i < num_joints; i++)
    {
      // Get trajectory data for position, velocity, acceleration
      std::vector<double> pos_data = f_x(time_points, [&, i](double t)
                                         {
        Eigen::VectorXd pos(num_joints); 
        trajectory->getState(t, &pos, nullptr, nullptr); 
        return pos[i]; });

      std::vector<double> vel_data = f_x(time_points, [&, i](double t)
                                         {
        Eigen::VectorXd vel(num_joints); 
        trajectory->getState(t, nullptr, &vel, nullptr); 
        return vel[i]; });

      std::vector<double> acc_data = f_x(time_points, [&, i](double t)
                                         {
        Eigen::VectorXd acc(num_joints); 
        trajectory->getState(t, nullptr, nullptr, &acc); 
        return acc[i]; });

      // Create subplots for position, velocity, and acceleration
      plt::figure();

      // Position subplot
      plt::subplot(3, 1, 1);
      plt::named_plot("Trajectory", time_points, pos_data, "-b");

      // Plot position waypoints
      std::vector<double> waypoint_times;
      std::vector<double> waypoint_positions;
      for (int j = 0; j < num_waypoints; j++)
      {
        waypoint_times.push_back(times(j));
        waypoint_positions.push_back(positions(i, j));
      }
      plt::named_plot("Waypoints", waypoint_times, waypoint_positions, "ko");

      // Plot min/max candidates
      for (int j = 0; j < pos_minmax_candidates.cols(); j++)
      {
        double maxval = pos_minmax_candidates(i, j);
        std::vector<double> x_values = {times(j), times(j+1)};
        std::vector<double> y_values = {maxval, maxval};
        // Add to legend only for the first candidate
        if (j == 0) {
          plt::named_plot("Min/Max Candidates", x_values, y_values, "k--");
        } else {
          plt::plot(x_values, y_values, "k--");
        }
      }

      plt::title("Position Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Position (rad)");
      plt::legend();

      // Velocity subplot
      plt::subplot(3, 1, 2);
      plt::named_plot("Trajectory", time_points, vel_data, "-g");
      
      // Plot velocity min/max candidates
      for (int j = 0; j < vel_minmax_candidates.cols(); j++)
      {
        double maxval = vel_minmax_candidates(i, j);
        std::vector<double> x_values = {times(j), times(j+1)};
        std::vector<double> y_values = {maxval, maxval};
        // Add to legend only for the first candidate
        if (j == 0) {
          plt::named_plot("Min/Max Candidates", x_values, y_values, "k--");
        } else {
          plt::plot(x_values, y_values, "k--");
        }
      }
      
      plt::title("Velocity Profile - Joint " + std::to_string(i + 1));
      plt::xlabel("Time (s)");
      plt::ylabel("Velocity (rad/s)");
      plt::legend();

      // Acceleration subplot
      plt::subplot(3, 1, 3);
      plt::named_plot("Trajectory", time_points, acc_data, "-r");
      
      // Plot acceleration min/max candidates
      for (int j = 0; j < acc_minmax_candidates.cols(); j++)
      {
        double maxval = acc_minmax_candidates(i, j);
        std::vector<double> x_values = {times(j), times(j+1)};
        std::vector<double> y_values = {maxval, maxval};
        // Add to legend only for the first candidate
        if (j == 0) {
          plt::named_plot("Min/Max Candidates", x_values, y_values, "k--");
        } else {
          plt::plot(x_values, y_values, "k--");
        }
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

  return 0;
}
