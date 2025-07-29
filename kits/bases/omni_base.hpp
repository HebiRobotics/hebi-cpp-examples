#pragma once

// HEBI C++ API files:
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"
#include "util/vector_utils.h"
#include "group.hpp"
#include "color.hpp"
#include <cmath>
#include <Eigen/Dense>

using namespace hebi;

// This struct represents the velocity of the chassis in 2D space with rotation around the Z-axis
struct ChassisVelocity {
    double x_;
    double y_;
    double rz_;
};


// This class represents a base with three omni wheels for movement in 2D space
class OmniBase {
public:
    static constexpr double WHEEL_RADIUS = 0.0762; // meters
    static constexpr double BASE_RADIUS = 0.220; // meters

    // Create omnibase and initialize the command
    OmniBase(std::shared_ptr<Group> group);

    // Evaluate Trajectory State and update commands
    bool update(const double t_now);
    void send() const;

	// Builds a smooth trajectory for the base to move to a target velocity
    void buildSmoothVelocityTrajectory(const double dx, const double dy, const double dtheta, const double t_now);
    void buildVelocityTrajectory(const Eigen::VectorXd& times, const Eigen::MatrixXd& velocities);

    std::shared_ptr<Group> group_;
    GroupCommand base_command_;

private:
    GroupFeedback base_feedback_;
    Color color_;
    std::shared_ptr<trajectory::Trajectory> trajectory_;
    Eigen::Matrix3d vels_base_to_wheel_;

    Eigen::Matrix3d buildJacobian(const double base_radius, const double wheel_radius);
};

