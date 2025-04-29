#include <iostream>
#include <Eigen/Dense>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <chrono>
#include <vector>
#include <cmath>

#include "trajectory.hpp"
#include "Eigen/Eigen"
#include <math.h>
#include "util/plot_functions.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace plt = matplotlibcpp;

bool planTrajectory(const mav_trajectory_generation::Vertex::Vector &vertices,
                    const std::vector<double> &v_max,
                    const std::vector<double> &a_max,
                    const std::vector<double> &initial_segment_times,
                    mav_trajectory_generation::Trajectory *trajectory = nullptr,
                    const bool segment_wise = false)
{
    if (vertices.empty()) {
        std::cerr << "No waypoints provided!" << std::endl;
        return false;
    }
    
    const int num_joints = vertices[0].D(); // Number of joints (dimensions) in the trajectory
    
    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 50;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 100.0;
    parameters.use_soft_constraints = true;
    parameters.soft_constraint_weight = 1.5;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    parameters.equality_constraint_tolerance = 1.0e-3;
    parameters.algorithm = nlopt::LD_LBFGS; // Using a gradient-based algorithm (Limited-memory BFGS)
    parameters.print_debug_info_time_allocation = true;
    parameters.print_debug_info = true;
    parameters.segment_wise = segment_wise;
    parameters.minimum_segment_time = 0.1;
    parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kMellingerOuterLoop;

    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;

    // estimate initial segment times
    std::vector<double> segment_times;
    // Use provided segment times instead of estimating
    segment_times = initial_segment_times;

    // Ensure segment_times are greater than or equal to the minimum segment time
    for (double &time : segment_times) {
        if (time < parameters.minimum_segment_time) {
            time = parameters.minimum_segment_time;
        }
    }

    double initial_total_time = 0;
    for (int i = 0; i < int(segment_times.size()); i++)
    {
      initial_total_time += segment_times.at(i);
    }
    std::cout << "Initial total time: " << initial_total_time << " seconds" << std::endl;

    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(num_joints, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration if specified
    for (int i = 0; i < num_joints; ++i) {
        if (!std::isnan(v_max[i]) || v_max[i] == std::numeric_limits<double>::infinity()) {
            opt.addMaximumMagnitudeConstraint(i, mav_trajectory_generation::derivative_order::VELOCITY, v_max[i]);
        }
        
        if (!std::isnan(a_max[i]) || a_max[i] == std::numeric_limits<double>::infinity()) {
            opt.addMaximumMagnitudeConstraint(i, mav_trajectory_generation::derivative_order::ACCELERATION, a_max[i]);
        }
    }

    std::cout << "Optimizing..." << std::endl;

    // solve trajectory
    opt.optimize();

    std::string result_str;

    switch (opt.getOptimizationInfo().stopping_reason)
    {
    case nlopt::FAILURE:
    {
      result_str = "generic failure";
      break;
    }
    case nlopt::INVALID_ARGS:
    {
      result_str = "invalid args";
      break;
    }
    case nlopt::OUT_OF_MEMORY:
    {
      result_str = "out of memory";
      break;
    }
    case nlopt::ROUNDOFF_LIMITED:
    {
      result_str = "roundoff limited";
      break;
    }
    case nlopt::FORCED_STOP:
    {
      result_str = "forced stop";
      break;
    }
    case nlopt::STOPVAL_REACHED:
    {
      result_str = "stopval reached";
      break;
    }
    case nlopt::FTOL_REACHED:
    {
      result_str = "ftol reached";
      break;
    }
    case nlopt::XTOL_REACHED:
    {
      result_str = "xtol reached";
      break;
    }
    case nlopt::MAXEVAL_REACHED:
    {
      result_str = "maxeval reached";
      break;
    }
    case nlopt::MAXTIME_REACHED:
    {
      result_str = "maxtime reached";
      break;
    }
    default:
    {
      result_str = "UNKNOWN FAILURE CODE";
      break;
    }
    }

    // Print number of iterations
    std::cout << "Number of iterations: " << opt.getOptimizationInfo().n_iterations << std::endl;

    if (opt.getOptimizationInfo().stopping_reason >= 1 && opt.getOptimizationInfo().stopping_reason != 6)
    {
        std::cout << "[TrajectoryGeneration]: optimization finished successfully with code " 
                  << opt.getOptimizationInfo().stopping_reason << ", '" << result_str << "'" << std::endl;
    }
    else if (opt.getOptimizationInfo().stopping_reason == -1)
    {
        std::cout << "[TrajectoryGeneration]: optimization finished with a generic error code " 
                  << opt.getOptimizationInfo().stopping_reason << ", '" << result_str << "'" << std::endl;
    }
    else
    {
        std::cerr << "[TrajectoryGeneration]: optimization failed with code " 
                  << opt.getOptimizationInfo().stopping_reason << ", '" << result_str << "'" << std::endl;
        return false;
    }

    // // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    // Print segment times and total trajectory time
    std::vector<double> new_segment_times = trajectory->getSegmentTimes();
    double total_time = 0.0;
    for (auto time : new_segment_times) {
        // std::cout << "Segment time: " << time << std::endl;
        total_time += time;
    }
    std::cout << "Total trajectory time: " << total_time << " seconds" << std::endl;

    return true;
}

int main(int argc, char **argv)
{
    /*----------- PARSE ARGUMENTS ------------*/
    // Default parameters
    int num_joints = 6;
    int num_waypoints = 10;
    bool no_plot = false;
    bool show_plot = false;
    bool save_plot = false;

    // Process command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--show-plot") {
            no_plot = false;
            show_plot = true;
        } else if (arg == "--save-plot") {
            no_plot = false;
            save_plot = true;
        } else if (arg == "--no-plot") {
            no_plot = true;
        } else if (arg == "--joints" && i + 1 < argc) {
            num_joints = std::stoi(argv[++i]);
            if (num_joints <= 0) {
                std::cerr << "Number of joints must be positive" << std::endl;
                return 1;
            }
        } else if (arg == "--waypoints" && i + 1 < argc) {
            num_waypoints = std::stoi(argv[++i]);
            if (num_waypoints < 2) {
                std::cerr << "Number of waypoints must be at least 2" << std::endl;
                return 1;
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --no-plot               Disable plotting" << std::endl;
            std::cout << "  --joints <number>       Set number of joints (default: 10)" << std::endl;
            std::cout << "  --waypoints <number>    Set number of waypoints (default: 10)" << std::endl;
            std::cout << "  --help, -h              Show this help message" << std::endl;
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            std::cerr << "Use --help for usage information" << std::endl;
            return 1;
        }
    }

    std::cout << "Running with " << num_joints << " joints, " 
              << num_waypoints << " waypoints, plotting " 
              << (no_plot ? "enabled" : "disabled") << std::endl;

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
    for (int i = 1; i < num_waypoints - 1; i++) {
        for (int j = 0; j < num_joints; j++) {
            positions(j, i) = ((double)rand() / RAND_MAX) * M_PI;
        }
        velocities.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
        accelerations.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
    }
    
    // End point with position and velocity constraints
    positions.col(num_waypoints-1) = Eigen::VectorXd::Constant(num_joints, M_PI_2);
    velocities.col(num_waypoints-1) = Eigen::VectorXd::Zero(num_joints);
    accelerations.col(num_waypoints-1) = Eigen::VectorXd::Zero(num_joints);

    // Set maximum velocity and acceleration
    std::vector<double> max_v(num_joints, 5.0);  // m/s
    std::vector<double> max_a(num_joints, 1000.0);  // m/s^2

    // Create vertices for trajectory optimization
    mav_trajectory_generation::Vertex::Vector vertices;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;

    // Create vertices for each waypoint
    for (int i = 0; i < num_waypoints; ++i) {
        mav_trajectory_generation::Vertex vertex(num_joints);
        
        // Make start and end points fixed for all derivatives
        if (i == 0 || i == num_waypoints - 1) {
            vertex.makeStartOrEnd(positions.col(i), derivative_to_optimize);
        }
        else {
            vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, positions.col(i));
        
            // Add velocity constraint if not NaN
            if (!velocities.col(i).array().isNaN().any()) {
                vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, velocities.col(i));
            }
            
            // Add acceleration constraint if not NaN
            if (!accelerations.col(i).array().isNaN().any()) {
                vertex.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, accelerations.col(i));
            }
        }
        
        vertices.push_back(vertex);
    }

    std::cout << "Vertices:" << std::endl;
    std::cout << vertices << std::endl;

    // Create initial segment times for all trajectories
    std::vector<double> initial_segment_times;
    initial_segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, max_v, max_a);
    // for (int i = 0; i < num_waypoints - 1; ++i) {
    //     initial_segment_times.push_back(0.5);
    // }

    /*----------- MAV TRAJECTORY (UNSEGMENTWISE) ------------*/
    
    // Generate MAV trajectory
    mav_trajectory_generation::Trajectory mav_traj;
    auto start_time = std::chrono::high_resolution_clock::now();
    bool mav_success = planTrajectory(vertices, max_v, max_a, 
                                     initial_segment_times, &mav_traj);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> mav_duration_ms = end_time - start_time;

    if (mav_success) {
        std::cout << "MAV trajectory planning successful!" << std::endl;
        std::cout << "MAV trajectory computation time: " << mav_duration_ms.count() << " milliseconds" << std::endl;
        std::cout << "MAV trajectory duration: " << mav_traj.getMaxTime() << " seconds" << std::endl;
    } else {
        std::cout << "MAV trajectory planning failed!" << std::endl;
        return 0;
    }

    /*----------- MAV TRAJECTORY (SEGMENTWISE) ------------*/

    mav_trajectory_generation::Trajectory mav_traj_segmentwise;
    start_time = std::chrono::high_resolution_clock::now();
    bool mav_segmentwise_success = planTrajectory(vertices, max_v, max_a,
                                                initial_segment_times, &mav_traj_segmentwise, true);
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> mav_segmentwise_duration_ms = end_time - start_time;

    if (mav_segmentwise_success) {
        std::cout << "MAV segment-wise trajectory planning successful!" << std::endl;
        std::cout << "MAV segment-wise trajectory computation time: " << mav_segmentwise_duration_ms.count() << " milliseconds" << std::endl;
        std::cout << "MAV segment-wise trajectory duration: " << mav_traj_segmentwise.getMaxTime() << " seconds" << std::endl;
    } else {
        std::cout << "MAV segment-wise trajectory planning failed!" << std::endl;
        return 0;
    }

    /*----------- HEBI TRAJECTORY ------------*/

    // Time vector based on the same initial segment times
    Eigen::VectorXd times(num_waypoints);
    times(0) = 0.0;
    for (int i = 1; i < num_waypoints; i++) {
        times(i) = times(i-1) + initial_segment_times[i-1];
    }

    // Generate HEBI trajectory
    start_time = std::chrono::high_resolution_clock::now();
    auto hebi_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> hebi_duration_ms = end_time - start_time;
    
    std::cout << "HEBI trajectory generation successful!" << std::endl;
    std::cout << "HEBI trajectory computation time: " << hebi_duration_ms.count() << " milliseconds" << std::endl;
    std::cout << "HEBI trajectory duration: " << hebi_traj->getDuration() << " seconds" << std::endl;

    // Plot trajectories if matplotlib-cpp is available
    if (!no_plot) {
        try {
            std::cout << "\nGenerating comparison plots..." << std::endl;
            
            // Set up time ranges for all trajectories
            double mav_duration = mav_traj.getMaxTime();
            double mav_segmentwise_duration = mav_traj_segmentwise.getMaxTime();
            double hebi_duration = hebi_traj->getDuration();
            double max_duration = std::max(std::max(mav_duration, mav_segmentwise_duration), hebi_duration);
            
            // Generate time points for plotting
            auto x_mav = linspace(0.0, mav_duration, 1000.0);
            auto x_mav_segmentwise = linspace(0.0, mav_segmentwise_duration, 1000.0);
            auto x_hebi = linspace(0.0, hebi_duration, 1000.0);
            
            for (int i = 0; i < num_joints; i++) {
                // MAV trajectory data
                std::vector<double> mav_pos = f_x(x_mav, [&, i](double t) {
                    Eigen::VectorXd point = mav_traj.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
                    return point[i]; 
                });
                
                std::vector<double> mav_vel = f_x(x_mav, [&, i](double t) {
                    Eigen::VectorXd point = mav_traj.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
                    return point[i]; 
                });
                
                std::vector<double> mav_acc = f_x(x_mav, [&, i](double t) {
                    Eigen::VectorXd point = mav_traj.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);
                    return point[i]; 
                });
                
                // MAV segmentwise trajectory data
                std::vector<double> mav_segmentwise_pos = f_x(x_mav_segmentwise, [&, i](double t) {
                    Eigen::VectorXd point = mav_traj_segmentwise.evaluate(t, mav_trajectory_generation::derivative_order::POSITION);
                    return point[i]; 
                });
                
                std::vector<double> mav_segmentwise_vel = f_x(x_mav_segmentwise, [&, i](double t) {
                    Eigen::VectorXd point = mav_traj_segmentwise.evaluate(t, mav_trajectory_generation::derivative_order::VELOCITY);
                    return point[i]; 
                });
                
                std::vector<double> mav_segmentwise_acc = f_x(x_mav_segmentwise, [&, i](double t) {
                    Eigen::VectorXd point = mav_traj_segmentwise.evaluate(t, mav_trajectory_generation::derivative_order::ACCELERATION);
                    return point[i]; 
                });
                
                // HEBI trajectory data
                std::vector<double> hebi_pos = f_x(x_hebi, [&, i](double t) {
                    Eigen::VectorXd pos(num_joints); 
                    hebi_traj->getState(t, &pos, nullptr, nullptr); 
                    return pos[i]; 
                });
                
                std::vector<double> hebi_vel = f_x(x_hebi, [&, i](double t) {
                    Eigen::VectorXd vel(num_joints); 
                    hebi_traj->getState(t, nullptr, &vel, nullptr); 
                    return vel[i]; 
                });
                
                std::vector<double> hebi_acc = f_x(x_hebi, [&, i](double t) {
                    Eigen::VectorXd acc(num_joints); 
                    hebi_traj->getState(t, nullptr, nullptr, &acc); 
                    return acc[i]; 
                });
                
                // Create subplots for position, velocity, and acceleration
                plt::figure();

                // Position subplot
                plt::subplot(3, 1, 1);
                plt::named_plot("MAV", x_mav, mav_pos, "-b");
                plt::named_plot("MAV Segmentwise", x_mav_segmentwise, mav_segmentwise_pos, "-.g");
                plt::named_plot("HEBI", x_hebi, hebi_pos, "--r");
                
                // Plot position waypoints
                std::vector<double> waypoint_times;
                std::vector<double> waypoint_positions;
                for (int j = 0; j < num_waypoints; j++) {
                    waypoint_times.push_back(times(j));
                    waypoint_positions.push_back(positions(i, j));
                }
                plt::named_plot("Waypoints", waypoint_times, waypoint_positions, "ko");
                
                // Mark MAV trajectory segment ends
                std::vector<double> segment_times = mav_traj.getSegmentTimes();
                std::vector<double> segment_end_times;
                std::vector<double> segment_end_positions;
                double accumulated_time = 0.0;
                
                for (size_t j = 0; j < segment_times.size(); j++) {
                    accumulated_time += segment_times[j];
                    segment_end_times.push_back(accumulated_time);
                    Eigen::VectorXd point = mav_traj.evaluate(accumulated_time, mav_trajectory_generation::derivative_order::POSITION);
                    segment_end_positions.push_back(point[i]);
                }
                plt::named_plot("MAV Segments", segment_end_times, segment_end_positions, "bs");
                
                plt::title("Position Comparison - Joint " + std::to_string(i+1));
                plt::xlabel("Time (s)");
                plt::ylabel("Position");
                plt::legend();

                // Velocity subplot
                plt::subplot(3, 1, 2);
                plt::named_plot("MAV", x_mav, mav_vel, "-b");
                plt::named_plot("MAV Segmentwise", x_mav_segmentwise, mav_segmentwise_vel, "-.g");
                plt::named_plot("HEBI", x_hebi, hebi_vel, "--r");
                
                // Add velocity limits as horizontal lines
                std::vector<double> vtime = {0, max_duration};
                std::vector<double> vupper = {max_v[i], max_v[i]};
                std::vector<double> vlower = {-max_v[i], -max_v[i]};
                plt::named_plot("Velocity Limit", vtime, vupper, "--k");
                plt::plot(vtime, vlower, "--k");
                
                plt::title("Velocity Comparison - Joint " + std::to_string(i+1));
                plt::xlabel("Time (s)");
                plt::ylabel("Velocity");
                plt::legend();
                
                // Acceleration subplot
                plt::subplot(3, 1, 3);
                plt::named_plot("MAV", x_mav, mav_acc, "-b");
                plt::named_plot("MAV Segmentwise", x_mav_segmentwise, mav_segmentwise_acc, "-.g");
                plt::named_plot("HEBI", x_hebi, hebi_acc, "--r");
                
                // // Add acceleration limits as horizontal lines
                // std::vector<double> atime = {0, max_duration};
                // std::vector<double> aupper = {max_a[i], max_a[i]};
                // std::vector<double> alower = {-max_a[i], -max_a[i]};
                // plt::named_plot("Acceleration Limit", atime, aupper, "--m");
                // plt::plot(atime, alower, "--m");
                
                plt::title("Acceleration Comparison - Joint " + std::to_string(i+1));
                plt::xlabel("Time (s)");
                plt::ylabel("Acceleration");
                plt::legend();

                // Show the combined plot
                if (save_plot) {
                    std::string filename = "trajectory_comparison_joint_" + std::to_string(i+1) + ".png";
                    plt::save(filename);
                    std::cout << "Saved plot to " << filename << std::endl;
                }
                if (show_plot) {
                    plt::show();
                }
            }
        } catch (const std::exception& e) {
            std::cout << "Note: plotting skipped - " << e.what() << std::endl;
        }
    } else {
        std::cout << "Plotting disabled by command line argument" << std::endl;
    }

    return 0;
}