#include "omni_base.hpp"


// Create omnibase and initialize the command
OmniBase::OmniBase(std::shared_ptr<Group> group) :
	velocities_(Eigen::MatrixXd::Zero(base_num_wheels_, 2)),
	accelerations_(Eigen::MatrixXd::Zero(base_num_wheels_, 2)),
	jerks_(Eigen::MatrixXd::Zero(base_num_wheels_, 2)),
	group_(group),
	base_command_(group->size()),
	color_(0, 0, 0) {

	GroupFeedback wheel_fbk(group_->size());
		
	while (!group_->getNextFeedback(wheel_fbk))
		std::cout << "Couldn't get feedback from the wheels!\n";

	// Initialize forward integration for position command, so we can update it later
	base_command_.setPosition(wheel_fbk.getPosition());
	last_time_ = wheel_fbk.getTime();
	traj_start_time_ = last_time_;

	// Initialize base trajectory
	const double base_ramp_time = .33;
	omni_base_traj_time_ << 0, base_ramp_time;
	trajectory_ = trajectory::Trajectory::createUnconstrainedQp(omni_base_traj_time_, velocities_, &accelerations_, &jerks_);
}

bool OmniBase::setGains()
{
	GroupCommand base_gains_command(group_->size());
	if (!base_gains_command.readGains("gains/omni-drive-wheel-gains.xml")) {
		std::cout << "Could not read omni base gains\n";
		return false;
	}
	if (!group_->sendCommandWithAcknowledgement(base_gains_command)) {
		std::cout << "Could not send omni base gains\n";
		return false;
	}
	return true;
}

// Evaluate Trajectory State and update commands
void OmniBase::update(double t, double x_vel, double y_vel, double rot_vel)
{
	double dt = t - last_time_;
	last_time_ = t;

	Eigen::VectorXd cmd_vel(base_num_wheels_), cmd_acc(base_num_wheels_), cmd_jerk(base_num_wheels_);
	trajectory_->getState(t - traj_start_time_, &cmd_vel, &cmd_acc, &cmd_jerk);

	// Build commands from trajectory
	base_command_.setVelocity(base_wheel_velocity_matrix_ * cmd_vel);
	base_command_.setPosition(base_command_.getPosition() + base_command_.getVelocity() * dt);
	base_command_.setEffort(base_wheel_effort_matrix_ * (chassis_mass_matrix_ * cmd_acc));

	for (int i = 0; i < base_num_wheels_; ++i)
		base_command_[i].led().set(color_);

	buildVelocityTrajectory(x_vel, y_vel, rot_vel, t);
}

void OmniBase::buildVelocityTrajectory(double x_vel, double y_vel, double rot_vel, double t) {

	// Duplicated in update; could cache this.
	Eigen::VectorXd chassis_cmd_vel(base_num_wheels_), chassis_cmd_acc(base_num_wheels_), chassis_cmd_jerk(base_num_wheels_);
	trajectory_->getState(t - traj_start_time_, &chassis_cmd_vel, &chassis_cmd_acc, &chassis_cmd_jerk);

	// Rebuild trajectory
	Eigen::Vector3d chassis_desired_vel;
	chassis_desired_vel << base_max_lin_speed_ * x_vel, base_max_lin_speed_ * y_vel, base_max_rot_speed_* rot_vel;

	velocities_.col(0) = chassis_cmd_vel;
	velocities_.col(1) = chassis_desired_vel;
	accelerations_.col(0) = chassis_cmd_acc;
	jerks_.col(0) = chassis_cmd_jerk;

	trajectory_ = trajectory::Trajectory::createUnconstrainedQp(omni_base_traj_time_, velocities_, &accelerations_, &jerks_);
	traj_start_time_ = t;
}

void OmniBase::send() {
	group_->sendCommand(base_command_);
}
	
void OmniBase::stop() {
	base_command_.setVelocity(Eigen::Vector3d::Zero());
}