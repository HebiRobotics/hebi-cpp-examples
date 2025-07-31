#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"
#include "color.hpp"

#include <algorithm>

using namespace hebi;

struct OmniBase {

	// Create omnibase and initialize the command
	OmniBase(std::shared_ptr<Group> group) :
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

	bool setGains()
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
	void update(double t, double x_vel, double y_vel, double rot_vel)
	{
		double dt = t - last_time_;
		last_time_ = t;

		Eigen::VectorXd cmd_vel(base_num_wheels_), cmd_acc(base_num_wheels_), cmd_jerk(base_num_wheels_);
		double traj_time = std::min(trajectory_->getDuration(), t - traj_start_time_);
		trajectory_->getState(traj_time, &cmd_vel, &cmd_acc, &cmd_jerk);

		// Build commands from trajectory
		base_command_.setVelocity(base_wheel_velocity_matrix_ * cmd_vel);
		base_command_.setPosition(base_command_.getPosition() + base_command_.getVelocity() * dt);
		base_command_.setEffort(base_wheel_effort_matrix_ * (chassis_mass_matrix_ * cmd_acc));

		for (int i = 0; i < base_num_wheels_; ++i)
			base_command_[i].led().set(color_);

		buildVelocityTrajectory(x_vel, y_vel, rot_vel, t);
	}

	void buildVelocityTrajectory(double x_vel, double y_vel, double rot_vel, double t) {

		// Duplicated in update; could cache this.
		Eigen::VectorXd chassis_cmd_vel(base_num_wheels_), chassis_cmd_acc(base_num_wheels_), chassis_cmd_jerk(base_num_wheels_);
		trajectory_->getState(t - traj_start_time_, &chassis_cmd_vel, &chassis_cmd_acc, &chassis_cmd_jerk);

		// Rebuild trajectory
		Eigen::Vector3d chassis_desired_vel;
		std::cout << "2. x_vel: " << x_vel << ", y_vel: " << y_vel << ", rot_vel: " << rot_vel << std::endl;
		chassis_desired_vel << base_max_lin_speed_ * x_vel, -base_max_lin_speed_ * y_vel, base_max_rot_speed_* rot_vel;
		std::cout << "3. chassis_desired_vel: " << chassis_desired_vel.transpose() << std::endl;

		velocities_.col(0) = chassis_cmd_vel;
		velocities_.col(1) = chassis_desired_vel;
		accelerations_.col(0) = chassis_cmd_acc;
		jerks_.col(0) = chassis_cmd_jerk;

		trajectory_ = trajectory::Trajectory::createUnconstrainedQp(omni_base_traj_time_, velocities_, &accelerations_, &jerks_);
		traj_start_time_ = t;
	}

	void send() {
		group_->sendCommand(base_command_);
	}
	
	void stop() {
		base_command_.setVelocity(Eigen::Vector3d::Zero());
	}

	std::shared_ptr<Group> group_;

private:

	double WHEEL_RADIUS = .15 / 2.0; //meters
	double BASE_RADIUS = 0.235; //meters

	double last_time_{};
	double traj_start_time_{};
	const int base_num_wheels_{ 3 };

	GroupCommand base_command_;
	Eigen::Vector2d omni_base_traj_time_;
	Eigen::MatrixXd velocities_;
	Eigen::MatrixXd accelerations_;
	Eigen::MatrixXd jerks_;
	std::shared_ptr<trajectory::Trajectory> trajectory_;
	Color color_;

	const double a1 = -60 * M_PI / 180;
	const double a2 = 60 * M_PI / 180;
	const double a3 = 180 * M_PI / 180;

	const double base_chassis_mass_ = 12.0;
	const double base_chassis_inertia_zz_ = .5 * base_chassis_mass_ * BASE_RADIUS * BASE_RADIUS;
	const Eigen::MatrixXd chassis_mass_matrix_
	{ (Eigen::MatrixXd(3,3) << base_chassis_mass_,0,0, 0,base_chassis_mass_,0, 0,0,base_chassis_inertia_zz_).finished() };

	const Eigen::Matrix3d base_wheel_transform_
	{ (Eigen::MatrixXd(3,3) <<
	 sin(a1), -cos(a1), -BASE_RADIUS,
	 sin(a2), -cos(a2), -BASE_RADIUS,
	 sin(a3), -cos(a3), -BASE_RADIUS).finished() };
	const Eigen::Matrix3d base_wheel_velocity_matrix_{ base_wheel_transform_ / WHEEL_RADIUS };
	const Eigen::Matrix3d base_wheel_effort_matrix_{ base_wheel_transform_ * WHEEL_RADIUS };

	const double base_max_lin_speed_ = 0.6;
	const double base_max_rot_speed_ = base_max_lin_speed_ * (BASE_RADIUS);
};

// This struct represents the velocity of the chassis in 2D space with rotation around the Z-axis
struct ChassisVelocity {
	double x_;
	double y_;
	double rz_;
};
