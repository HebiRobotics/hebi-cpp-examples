#include <nlopt.hpp>
#include <vector>
#include <iostream>
#include "util/plot_functions.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace plt = matplotlibcpp;

std::vector<double> waypoints = {0.0, 3.0, 7.0, 10.0};  // Example positions
const double w_time = 1.0, w_jerk = 0.1;  // Optimization weights

// Objective function: minimize time and jerk
double objective(unsigned n, const double* x, double* grad, void* data) {
    double total = 0.0;
    // Sum of time intervals
    for (size_t i = 0; i < n/2; ++i) {
        total += w_time * x[2*i];  // Time weight * dt
        total += w_jerk * x[2*i+1] * x[2*i+1];  // Jerk weight * j^2
    }
    
    // Compute gradient if requested
    if (grad) {
        for (size_t i = 0; i < n/2; ++i) {
            grad[2*i] = w_time;  // Derivative with respect to dt
            grad[2*i+1] = 2.0 * w_jerk * x[2*i+1];  // Derivative with respect to jerk
        }
    }
    
    return total;
}

struct ConstraintData { int segment; double target_pos; };

// Position constraints for each segment - modified to match NLopt's expected signature
double position_constraint(unsigned n, const double* x, double* grad, void* data) {
    auto* cd = static_cast<ConstraintData*>(data);
    double p = waypoints[0], v = 0, a = 0;
    
    // Calculate position at the end of the segment
    for (int i = 0; i <= cd->segment; ++i) {
        double dt = x[2*i], j = x[2*i+1];
        p += v*dt + 0.5*a*dt*dt + (j*dt*dt*dt)/6;
        v += a*dt + 0.5*j*dt*dt;
        a += j*dt;
    }
    
    // Compute gradient if requested
    if (grad) {
        std::fill(grad, grad + n, 0.0); // Initialize all gradients to zero
        
        // Reset state to compute gradients
        double pgrad = waypoints[0], vgrad = 0, agrad = 0;
        
        for (int i = 0; i <= cd->segment; ++i) {
            double dt = x[2*i], j = x[2*i+1];
            
            // Gradient with respect to dt
            grad[2*i] = vgrad + agrad*dt + j*dt*dt/2;
            
            // Gradient with respect to jerk
            grad[2*i+1] = dt*dt*dt/6;
            
            // Update state for next segment
            pgrad += vgrad*dt + 0.5*agrad*dt*dt + (j*dt*dt*dt)/6;
            vgrad += agrad*dt + 0.5*j*dt*dt;
            agrad += j*dt;
        }
    }
    
    return p - cd->target_pos;
}

// Position, velocity, and acceleration calculation at time t
void evaluate_state(const std::vector<double>& x, double t, double& pos, double& vel, double& acc) {
    pos = waypoints[0];
    vel = 0.0;
    acc = 0.0;
    double elapsed = 0.0;
    
    for (size_t i = 0; i < x.size()/2; ++i) {
        double dt = x[2*i];
        double jerk = x[2*i+1];
        
        if (t <= elapsed + dt) {
            // We're in this segment
            double segment_t = t - elapsed;
            pos += vel * segment_t + 0.5 * acc * segment_t * segment_t + (jerk * segment_t * segment_t * segment_t) / 6.0;
            vel += acc * segment_t + 0.5 * jerk * segment_t * segment_t;
            acc += jerk * segment_t;
            return;
        }
        
        // Move to next segment
        pos += vel * dt + 0.5 * acc * dt * dt + (jerk * dt * dt * dt) / 6.0;
        vel += acc * dt + 0.5 * jerk * dt * dt;
        acc += jerk * dt;
        elapsed += dt;
    }
}

int main() {
    const int M = waypoints.size()-1;  // Number of segments
    nlopt::opt opt(nlopt::LD_SLSQP, 2*M);
    
    // Set bounds (dt > 0)
    std::vector<double> lb(2*M);
    for (int i = 0; i < M; ++i) {
        lb[2*i] = 0.1;     // Minimum time interval (avoid near-zero dt)
        lb[2*i+1] = -100;  // Allow negative jerk values
    }
    opt.set_lower_bounds(lb);
    
    // Set upper bounds to prevent extremely large values
    std::vector<double> ub(2*M);
    for (int i = 0; i < M; ++i) {
        ub[2*i] = 10.0;    // Maximum time interval
        ub[2*i+1] = 100;   // Maximum jerk
    }
    opt.set_upper_bounds(ub);
    
    // Configure optimization
    opt.set_min_objective(objective, nullptr);
    opt.set_xtol_rel(1e-6);    // Tighter relative tolerance
    opt.set_ftol_rel(1e-6);    // Function value tolerance
    opt.set_maxeval(1000);     // Maximum number of evaluations
    
    // Create and store constraint data to ensure proper cleanup
    std::vector<ConstraintData*> constraint_data;
    
    // Add position constraints - modified to use NLopt's function type
    for (int i = 0; i < M; ++i) {
        auto* data = new ConstraintData{i, waypoints[i+1]};
        constraint_data.push_back(data);
        opt.add_equality_constraint(position_constraint, data, 1e-6); // Relaxed tolerance
    }

    // Initial guess: better estimate based on distance between waypoints
    std::vector<double> x(2*M);
    for (size_t i = 0; i < M; ++i) {
        // Estimate time based on distance between waypoints
        double distance = std::abs(waypoints[i+1] - waypoints[i]);
        x[2*i] = 0.5 + 0.5 * distance; // Base time + distance-dependent component
        
        // Initial jerk estimate
        x[2*i+1] = (i % 2 == 0) ? 0.5 : -0.5; // Alternating jerk directions
    }

    // Execute optimization
    try {
        double minf;
        nlopt::result result = opt.optimize(x, minf);
        
        std::cout << "Optimization result: ";
        switch (result) {
            case nlopt::SUCCESS: std::cout << "Success!\n"; break;
            case nlopt::STOPVAL_REACHED: std::cout << "Stop value reached\n"; break;
            case nlopt::FTOL_REACHED: std::cout << "Function tolerance reached\n"; break;
            case nlopt::XTOL_REACHED: std::cout << "Variable tolerance reached\n"; break;
            case nlopt::MAXEVAL_REACHED: std::cout << "Max evaluations reached\n"; break;
            case nlopt::MAXTIME_REACHED: std::cout << "Max time reached\n"; break;
            case nlopt::ROUNDOFF_LIMITED: std::cout << "Roundoff limited (but results may still be usable)\n"; break;
            default: std::cout << "Other termination code: " << result << "\n"; break;
        }
        
        std::cout << "Optimized trajectory:\n";
        double total_time = 0.0;
        for (int i = 0; i < M; ++i) {
            total_time += x[2*i];
            std::cout << "Segment " << i+1 << ": dt=" << x[2*i] 
                      << "s, jerk=" << x[2*i+1] << "m/s³\n";
        }
        std::cout << "Total trajectory time: " << total_time << "s\n";
        std::cout << "Minimum objective value: " << minf << "\n";

        // Plot the optimized trajectory
        try {
            std::cout << "\nGenerating trajectory plots...\n";
            
            // Generate time points for plotting
            auto time_points = linspace(0.0, total_time, 1000);
            
            // Calculate trajectory states at each time point
            std::vector<double> positions, velocities, accelerations;
            for (double t : time_points) {
                double pos, vel, acc;
                evaluate_state(x, t, pos, vel, acc);
                positions.push_back(pos);
                velocities.push_back(vel);
                accelerations.push_back(acc);
            }
            
            // Create waypoints for plotting
            std::vector<double> waypoint_times;
            double t = 0;
            for (int i = 0; i < M; i++) {
                waypoint_times.push_back(t);
                t += x[2*i]; // Add segment time
            }
            waypoint_times.push_back(t); // Add final waypoint time
            
            // Create position subplot
            plt::figure();
            
            plt::subplot(3, 1, 1);
            plt::named_plot("Position", time_points, positions, "-b");
            plt::named_plot("Waypoints", waypoint_times, waypoints, "ro");
            plt::title("Position Trajectory");
            plt::xlabel("Time (s)");
            plt::ylabel("Position");
            plt::grid(true);
            plt::legend();
            
            // Create velocity subplot
            plt::subplot(3, 1, 2);
            plt::named_plot("Velocity", time_points, velocities, "-g");
            plt::title("Velocity Profile");
            plt::xlabel("Time (s)");
            plt::ylabel("Velocity");
            plt::grid(true);
            
            // Create acceleration subplot
            plt::subplot(3, 1, 3);
            plt::named_plot("Acceleration", time_points, accelerations, "-r");
            plt::title("Acceleration Profile");
            plt::xlabel("Time (s)");
            plt::ylabel("Acceleration");
            plt::grid(true);
            
            // Show the combined plot
            plt::tight_layout();
            plt::show();
            
        } catch (const std::exception& e) {
            std::cout << "Note: plotting skipped - " << e.what() << std::endl;
        }
        
    } catch (std::exception& e) {
        std::cerr << "Optimization failed: " << e.what() << "\n";
    }
    
    // Clean up constraint data
    for (auto* data : constraint_data) {
        delete data;
    }

    return 0;
}
