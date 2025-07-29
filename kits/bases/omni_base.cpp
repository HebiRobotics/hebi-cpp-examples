#include "omni_base.hpp"


OmniBase::OmniBase(std::shared_ptr<Group> group) :
    group_(group),
    base_command_(group->size()),
    base_feedback_(group->size()),
    color_(0, 0, 0),
    trajectory_(nullptr),
    vels_base_to_wheel_(buildJacobian(BASE_RADIUS, WHEEL_RADIUS)) {
}

// Evaluate Trajectory State and update commands
bool OmniBase::update(const double t_now) {
    if (!group_->getNextFeedback(base_feedback_))
        return false;

    if (trajectory_) {
        Eigen::VectorXd p(group_->size()), v(group_->size()), a(group_->size());
        trajectory_->getState(t_now, &p, &v, &a);

        const double theta = p[2];

        Eigen::Matrix3d world_to_local_rot = Eigen::Matrix3d::Zero();
        world_to_local_rot(0, 0) = std::cos(theta);
        world_to_local_rot(0, 1) = -std::sin(theta);
        world_to_local_rot(1, 0) = std::sin(theta);
        world_to_local_rot(1, 1) = std::cos(theta);
        world_to_local_rot(2, 2) = 1.0;

        Eigen::Vector3d v_local = world_to_local_rot * v;
        base_command_.setVelocity(vels_base_to_wheel_ * v_local);

        for (int i = 0; i < group_->size(); ++i)
            base_command_[i].led().set(color_);
    }
    return true;
}

void OmniBase::send() const {
    group_->sendCommand(base_command_);
}

// Builds a smooth trajectory for the base to move to a target velocity
void OmniBase::buildSmoothVelocityTrajectory(const double dx, const double dy, const double dtheta, const double t_now) {
    Eigen::Vector4d times{ 0, 0.15, 0.9, 1.2 };

    Eigen::Vector3d cmd_vels = Eigen::Vector3d::Zero();
    if (trajectory_) {
        Eigen::VectorXd pos(group_->size()), vel(group_->size()), acc(group_->size());
        trajectory_->getState(t_now, &pos, &vel, &acc);
        cmd_vels = vel;
    }

    Eigen::Vector3d target_vel(dx, dy, dtheta);
    Eigen::MatrixXd velocities(3, 4);
    velocities.col(0) = cmd_vels;
    velocities.col(1) = target_vel;
    velocities.col(2) = target_vel;
    velocities.col(3) = Eigen::Vector3d::Zero();

    buildVelocityTrajectory(times.array() + t_now, velocities);
}

void OmniBase::buildVelocityTrajectory(const Eigen::VectorXd& times, const Eigen::MatrixXd& velocities) {
    Eigen::MatrixXd p = Eigen::MatrixXd::Constant(3, 4, std::nan(""));
    p.col(0) = Eigen::Vector3d::Zero();

    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(3, 4);
    trajectory_ = trajectory::Trajectory::createUnconstrainedQp(times, p, &velocities, &a);
}

Eigen::Matrix3d OmniBase::buildJacobian(const double base_radius, const double wheel_radius) {
    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();

    jacobian(0, 0) = -std::sqrt(3.0) / 2.0;
    jacobian(1, 0) = std::sqrt(3.0) / 2.0;
    jacobian(2, 0) = 0.0;

    jacobian(0, 1) = -0.5;
    jacobian(1, 1) = -0.5;
    jacobian(2, 1) = 1.0;

    jacobian.col(2) = Eigen::Vector3d::Constant(-base_radius);
    jacobian /= wheel_radius;

    return jacobian;
}