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

bool planTrajectory(const Eigen::MatrixXd &positions,
                    const Eigen::MatrixXd &velocities,
                    const Eigen::MatrixXd &accelerations,
                    double max_v = 10.0,
                    double max_a = 10.0,
                    mav_trajectory_generation::Trajectory *trajectory = nullptr)
{
    if (positions.cols() == 0) {
        std::cerr << "No waypoints provided!" << std::endl;
        return false;
    }
    
    const int num_joints = positions.rows();
    const int num_waypoints = positions.cols();
    
    // Check that all constraint matrices have the same size
    if (velocities.cols() != num_waypoints || accelerations.cols() != num_waypoints) {
        std::cerr << "Mismatch in constraint matrix sizes!" << std::endl;
        return false;
    }

    // Array for all waypoints and their constrains
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

    // estimate initial segment times
    std::vector<double> segment_times;
    // segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, max_v, max_a);
    // segment_times = {5.0, 5.0};
    for (int i = 0; i < num_waypoints - 1; ++i) {
        segment_times.push_back(0.5);
    }

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    // parameters.x_rel = 0.1;
    // parameters.time_penalty = 500.0;
    // parameters.inequality_constraint_tolerance = 0.1;
    // parameters.use_soft_constraints = false;
    parameters.algorithm = nlopt::LD_LBFGS; // Using a gradient-based algorithm (Limited-memory BFGS)
    parameters.print_debug_info_time_allocation = true;
    parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kMellingerOuterLoop;

    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(num_joints, parameters);
    // mav_trajectory_generation::PolynomialOptimization<N> opt(num_joints);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration if specified
    if (!std::isnan(max_v)) {
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v);
    }
    
    if (!std::isnan(max_a)) {
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a);
    }

    std::cout << "Optimizing..." << std::endl;

    // // solve trajectory
    opt.optimize();

    // opt.solveLinear();

    // Print number of iterations
    std::cout << "Number of iterations: " << opt.getOptimizationInfo().n_iterations << std::endl;

    // // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    // Print segment times of optimized trajectory
    // std::vector<double> new_segment_times = trajectory->getSegmentTimes();
    // for (auto time : new_segment_times) {
    //     std::cout << "Segment time: " << time << std::endl;
    // }

    return true;
}

int main(int argc, char **argv)
{
    /*----------- COMMON ------------*/
    // Define trajectory parameters
    const int num_joints = 1;
    const int num_waypoints = 10; // Increased to 100 waypoints
    
    // Create waypoint matrices for both trajectory generators
    Eigen::MatrixXd positions(num_joints, num_waypoints);
    Eigen::MatrixXd velocities(num_joints, num_waypoints);
    Eigen::MatrixXd accelerations(num_joints, num_waypoints);
    
    // Start point with full constraints
    positions.col(0) = Eigen::VectorXd::Zero(num_joints);
    velocities.col(0) = Eigen::VectorXd::Zero(num_joints);
    accelerations.col(0) = Eigen::VectorXd::Zero(num_joints);
    
    // Generate random waypoints between 0 and M_PI for intermediate points
    srand(time(NULL)); // Seed the random number generator
    for (int i = 1; i < num_waypoints - 1; i++) {
        positions.col(i) << ((double)rand() / RAND_MAX) * M_PI;
        velocities.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
        accelerations.col(i) = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
    }
    
    // End point with position and velocity constraints
    positions.col(num_waypoints-1) << M_PI_2;
    velocities.col(num_waypoints-1) = Eigen::VectorXd::Zero(num_joints);
    accelerations.col(num_waypoints-1) = Eigen::VectorXd::Zero(num_joints);

    /*----------- MAV TRAJECTORY ------------*/
    
    // Set maximum velocity and acceleration
    double max_v = 5.0;  // m/s
    double max_a = 1000.0;  // m/s^2
    
    // Generate MAV trajectory
    mav_trajectory_generation::Trajectory mav_traj;
    auto start_time = std::chrono::high_resolution_clock::now();
    bool mav_success = planTrajectory(positions, velocities, accelerations, max_v, max_a, &mav_traj);
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

    /*----------- HEBI TRAJECTORY ------------*/

    // Time vector
    Eigen::VectorXd times(num_waypoints);
    // times << 0.0, 5.0, 10.0;
    double avg_time = mav_traj.getMaxTime() / (num_waypoints - 1);
    for (int i = 0; i < num_waypoints; i++) {
        times(i) = i * avg_time;
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
    try {
        std::cout << "\nGenerating comparison plots..." << std::endl;
        
        // Set up time ranges for both trajectories
        double mav_duration = mav_traj.getMaxTime();
        double hebi_duration = hebi_traj->getDuration();
        double max_duration = std::max(mav_duration, hebi_duration);
        
        // Generate time points for plotting
        auto x_mav = linspace(0.0, mav_duration, 1000.0);
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
            plt::named_plot("HEBI", x_hebi, hebi_pos, "--r");
            
            // Plot position waypoints
            std::vector<double> waypoint_times;
            std::vector<double> waypoint_positions;
            for (int j = 0; j < num_waypoints; j++) {
                waypoint_times.push_back(times(j));
                waypoint_positions.push_back(positions(i, j));
            }
            plt::named_plot("Waypoints", waypoint_times, waypoint_positions, "go");
            
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
            plt::named_plot("HEBI", x_hebi, hebi_vel, "--r");
            plt::title("Velocity Comparison - Joint " + std::to_string(i+1));
            plt::xlabel("Time (s)");
            plt::ylabel("Velocity");
            plt::legend();
            
            // Acceleration subplot
            plt::subplot(3, 1, 3);
            plt::named_plot("MAV", x_mav, mav_acc, "-b");
            plt::named_plot("HEBI", x_hebi, hebi_acc, "--r");
            plt::title("Acceleration Comparison - Joint " + std::to_string(i+1));
            plt::xlabel("Time (s)");
            plt::ylabel("Acceleration");
            plt::legend();

            // Show the combined plot
            plt::show();
        }
    } catch (const std::exception& e) {
        std::cout << "Note: plotting skipped - " << e.what() << std::endl;
    }

    return 0;
}