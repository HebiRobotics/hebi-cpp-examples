// HEBI C++ API files:
#include "arm/arm.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"
#include "util/mobile_io.hpp"
#include "util/vector_utils.h"
#include "group.hpp"
#include "color.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <thread>
#include <chrono>

// Common includes
#include <iostream>

using namespace hebi;

class OmniBase {
public:
    static constexpr double WHEEL_RADIUS = 0.0762; // meters
    static constexpr double BASE_RADIUS = 0.220; // meters

    OmniBase(std::shared_ptr<Group> group) : 
        group_(group),
        base_command_(group->size()),
        base_feedback_(group->size()),
        color_(0, 0, 0), 
        trajectory_(nullptr),
        vels_base_to_wheel_(buildJacobian(BASE_RADIUS, WHEEL_RADIUS)) {}

    bool update(const double t_now) {
        if (!group_->getNextFeedback(base_feedback_))
            return false;

        std::cout << "Hey 12.1\n";
        if (trajectory_) {
            std::cout << "Hey 12.2\n";
            Eigen::VectorXd p(group_->size()), v(group_->size()), a(group_->size());
            double x = 10.5;
            trajectory_->getState(x, &p, &v, &a);
            std::cout << "Hey 12.3\n";

            const double theta = p[2];

            Eigen::Matrix3d world_to_local_rot = Eigen::Matrix3d::Zero();
            world_to_local_rot(0, 0) = std::cos(theta);
            world_to_local_rot(0, 1) = -std::sin(theta);
            world_to_local_rot(1, 0) = std::sin(theta);
            world_to_local_rot(1, 1) = std::cos(theta);
            world_to_local_rot(2, 2) = 1.0;

            Eigen::Vector3d v_local = world_to_local_rot * v;
            base_command_.setVelocity(vels_base_to_wheel_ * v_local);
	        base_command_[0].led().set(color_);
        }

        return true;
    }

    void send() const {
        group_->sendCommand(base_command_);
    }

    void buildSmoothVelocityTrajectory(const double dx, const double dy, const double dtheta, const double t_now) {
        Eigen::Vector4d times{0, 0.15, 0.9, 1.2};

        Eigen::Vector3d cmd_vels = Eigen::Vector3d::Zero();
        std::cout << "group size " << group_->size() << std::endl;
        std::cout << "Is error 1?\n";
        if (trajectory_) {
            std::cout << "Is error 2?\n";
            Eigen::VectorXd pos(group_->size()), vel(group_->size()), acc(group_->size());
            std::cout << "Position:\n" << pos << std::endl;
            std::cout << "Velocity:\n" << vel << std::endl;
            std::cout << "Acceleration:\n" << acc << std::endl;
            std::cout << "Is error 2.1?\n";
            trajectory_->getState(t_now, &pos, &vel, &acc);
            std::cout << "Is error 2.2?\n";
            cmd_vels = vel;
            std::cout << "Is error 3?\n";
        }

        std::cout << "Is error 4?\n";
        Eigen::Vector3d target_vel(dx, dy, dtheta);
        Eigen::MatrixXd velocities(3, 4);
        velocities.col(0) = cmd_vels;
        velocities.col(1) = target_vel;
        velocities.col(2) = target_vel;
        velocities.col(3) = Eigen::Vector3d::Zero();
        std::cout << "Is error 5?\n";

        buildVelocityTrajectory(times.array() + t_now, velocities);
        std::cout << "Is error 10?\n";
    }

    void buildVelocityTrajectory(const Eigen::VectorXd& times, const Eigen::MatrixXd& velocities) {
        std::cout << "Is error 6?\n";
        Eigen::MatrixXd p = Eigen::MatrixXd::Constant(3, 4, std::nan(""));
        p.col(0) = Eigen::Vector3d::Zero();
        std::cout << "Is error 7?\n";

        Eigen::MatrixXd a = Eigen::MatrixXd::Zero(3, 4);
        std::cout << "Is error 8?\n";
        trajectory_ = trajectory::Trajectory::createUnconstrainedQp(times, p, &velocities, &a);
        std::cout << "Is error 9?\n";
    }

    std::shared_ptr<Group> group_;
    GroupCommand base_command_;

private:
    GroupFeedback base_feedback_;
    Color color_;
    std::shared_ptr<trajectory::Trajectory> trajectory_;
    Eigen::Matrix3d vels_base_to_wheel_;

    Eigen::Matrix3d buildJacobian(const double base_radius, const double wheel_radius) {
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
};

struct ChassisVelocity {
    float x_;
    float y_;
    float rz_;

    ChassisVelocity(const float x = 0.0f, const float y = 0.0f, const float rz = 0.0f) : x_(x), y_(y), rz_(rz) {}

    std::string getInfo() const
    {
        std::string info = "ChassisVelocity : x=" + std::to_string(x_) + ", y=" + std::to_string(y_) + ", rz=" + std::to_string(rz_);
        return info;
    }
};

enum class ArmControlState { STARTUP, HOMING, TELEOP, DISCONNECTED, EXIT };

struct ArmMobileIOInputs {
    hebi::Vector3f phone_pos;
    Eigen::Matrix3d phone_rot;
    double ar_scaling;
    bool lock_toggle;
    bool locked;
    bool gripper_closed;
    bool home;

    // Optional constructor if you want to set values on creation
    ArmMobileIOInputs(
            const hebi::Vector3f& pos = hebi::Vector3f(0.0f, 0.0f, 0.0f), 
            const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), 
            const double scaling = 1.0f, 
            bool lockToggle = false, 
            bool isLocked = true,
            bool gripperClosed = false, 
            bool isHome = false) :
        phone_pos(pos),
        phone_rot(rot),
        ar_scaling(scaling),
        lock_toggle(lockToggle),
        locked(isLocked),
        gripper_closed(gripperClosed),
        home(isHome) {}
};

class ArmMobileIOControl
{
public:
    std::string namespace_ = "";
    ArmControlState state_ = ArmControlState::STARTUP;
    std::shared_ptr<arm::Arm> arm_ = nullptr;
    std::shared_ptr<arm::Gripper> gripper_ = nullptr;
    Eigen::Vector3d arm_xyz_home_ = { 0.5, 0.0, 0.0 };
    Eigen::Matrix3d arm_rot_home_;
    double homing_time_ = 5.0;
    double traj_duration_ = 1.0;
    Eigen::VectorXd arm_seed_ik_;
    Eigen::VectorXd arm_home_;

    Eigen::Vector3d xyz_scale_ = { 1.0, 1.0, 1.0 };
    Eigen::Vector3d last_locked_xyz_;
    Eigen::Matrix3d last_locked_rot_;
    Eigen::VectorXd last_locked_seed_;

    bool locked_ = true;
    double mobile_last_fbk_t_;

    std::vector<std::function<void(ArmMobileIOControl&, ArmControlState)>> transition_handlers_;

    hebi::Vector3f phone_xyz_home_{ 0.0, 0.0, 0.0 };
    Eigen::Matrix3d phone_rot_home_;

    ArmMobileIOControl(std::shared_ptr<arm::Arm> arm, std::shared_ptr<arm::Gripper> gripper, double homing_time, double traj_duration, Eigen::Vector3d xyz_scale) : arm_(arm), gripper_(gripper), homing_time_(homing_time), traj_duration_(traj_duration), xyz_scale_(xyz_scale) {
        arm_seed_ik_.resize(6); 
        arm_seed_ik_ << 0.3, 1.2, 2.2, 2.9, -1.57, 0;
        Eigen::AngleAxisd rotZ(M_PI / 2, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rotX(M_PI, Eigen::Vector3d::UnitX());
        arm_rot_home_ = (rotZ * rotX).toRotationMatrix();

        arm_home_ = arm_->solveIK(arm_seed_ik_, arm_xyz_home_, arm_rot_home_);
        last_locked_xyz_ = arm_xyz_home_;
        last_locked_rot_ = arm_rot_home_;
        last_locked_seed_ = arm_home_;

        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        mobile_last_fbk_t_ = time_now;
    }

    bool running() const { return state_ != ArmControlState::EXIT; }

    void send() const {

        std::cout << "Hey 4.3\n";
        arm_->send();
        std::cout << "Hey 4.4\n";
        if (gripper_) {

            std::cout << "Hey 4.5\n";
            gripper_->send();
            std::cout << "Hey 4.6\n";
        }
    }

    void home(const double duration) const {
        arm_->setGoal(arm::Goal::createFromPosition(duration, arm_home_));
    }

    void transition_to(const double t_now, ArmControlState new_state) {
        if (new_state == state_) 
            return;

        switch (new_state) {

            case ArmControlState::HOMING: 
                std::cout << namespace_ << " TRANSITION TO HOMING\n"; 
                home(homing_time_); 
                break;

            case ArmControlState::TELEOP: 
                std::cout << namespace_ << " TRANSITION TO TELEOP\n";
                break;

            case ArmControlState::DISCONNECTED: 
                std::cout << namespace_ << " MobileIO timeout, disabling motion\n";
                break;

            case ArmControlState::EXIT: 
                std::cout << namespace_ << " TRANSITION TO EXIT\n"; 
                break;

            default: 
                break;
        }

        for (auto& handler : transition_handlers_) 
            handler(*this, new_state);

        state_ = new_state;
    }

    arm::Goal compute_arm_goal(const ArmMobileIOInputs& arm_input) {
        Eigen::Vector3d phone_offset(
            arm_input.phone_pos.getX() - phone_xyz_home_.getX(),
            arm_input.phone_pos.getY() - phone_xyz_home_.getY(),
            arm_input.phone_pos.getZ() - phone_xyz_home_.getZ());

        auto rot_mat = phone_rot_home_;
        auto arm_xyz_target = last_locked_xyz_ + arm_input.ar_scaling * xyz_scale_.cwiseProduct(rot_mat.transpose() * phone_offset);
        Eigen::Matrix3d arm_rot_target = rot_mat.transpose() * arm_input.phone_rot * last_locked_rot_;

        if (arm_input.ar_scaling == 0.0)
            phone_xyz_home_ = arm_input.phone_pos;
        const auto joint_target = arm_->solveIK(last_locked_seed_, arm_xyz_target, arm_rot_target);
        const arm::Goal goal = arm::Goal::createFromPosition(traj_duration_, joint_target);

        return goal;
    }

    void update(const double t_now, const ArmMobileIOInputs* arm_input) {
        arm_->update();

        if (state_ == ArmControlState::EXIT)
             return;

        if (!arm_input)
        {
            if (state_ != ArmControlState::DISCONNECTED && t_now - mobile_last_fbk_t_ > 1.0) {
                std::cout << namespace_ << " mobileIO timeout, disabling motion\n";
                transition_to(t_now, ArmControlState::DISCONNECTED);
            }
            return;
        }

        mobile_last_fbk_t_ = t_now;
        auto last_pos = arm_->lastFeedback().getPosition();

        switch (state_) {

        case ArmControlState::DISCONNECTED:
            mobile_last_fbk_t_ = t_now;
            std::cout << namespace_ << " Controller reconnected, demo continued\n";
            locked_ = true;
            transition_to(t_now, ArmControlState::TELEOP);
            break;

        case ArmControlState::STARTUP:
            transition_to(t_now, ArmControlState::HOMING);
            break;

        case ArmControlState::HOMING:
            if (arm_->atGoal()) {
                phone_xyz_home_ = arm_input->phone_pos;
                phone_rot_home_ = arm_input->phone_rot;

                last_locked_seed_ = last_pos;
                arm_->FK(last_pos, last_locked_xyz_, last_locked_rot_);
                transition_to(t_now, ArmControlState::TELEOP);
            }
            break;

        case ArmControlState::TELEOP:

            if (arm_input->home) {
                 transition_to(t_now, ArmControlState::HOMING);
                 return;
            }

            if (arm_input->lock_toggle)
                locked_ = arm_input->locked;

            if (!locked_) {
                const arm::Goal arm_goal = compute_arm_goal(*arm_input);
                arm_->setGoal(arm_goal);
            }
            else {
                phone_xyz_home_ = arm_input->phone_pos;
                phone_rot_home_ = arm_input->phone_rot;
                last_locked_seed_ = last_pos;
                arm_->FK(last_pos, last_locked_xyz_, last_locked_rot_);
            }

            if (gripper_) {
                auto gripper_closed = gripper_->getState() == 1.0;
                if (arm_input->gripper_closed && !gripper_closed) {
                    std::cout << "Gripper Close";
                    gripper_->close();
                }
                else if (!arm_input->gripper_closed && gripper_closed) {
                    std::cout << "Gripper Open";
                    gripper_->open();
                }
            }
            break;
        }
}

    void stop() {
        auto now = std::chrono::system_clock::now();
        std::time_t time_now = std::chrono::system_clock::to_time_t(now);
        transition_to(time_now, ArmControlState::EXIT);
    }
};

OmniBase setupBase(const Lookup& lookup, const std::string& base_family) {
    const std::vector<std::string> wheel_names = { "W1", "W2", "W3" };

    auto wheel_group = lookup.getGroupFromNames({ base_family }, wheel_names);
    std::cout << "Wheel group size\n" << wheel_group->size() <<std::endl;
    if (!wheel_group) {
        throw std::runtime_error("Could not find wheel modules: \"W1\", \"W2\", \"W3\" in family '" + base_family + "'");
    }

    auto base = OmniBase(wheel_group);

    return base;
}

void setMobileIOInstructions(util::MobileIO& mobile_io, const std::string& message, const Color& color = Color{0,0,0}) {

    mobile_io.setLedColor(color.getRed(), color.getGreen(), color.getBlue(), false);
    mobile_io.clearText(false);
    mobile_io.appendText(message, false);

    std::cout << message << std::endl;
}

void setupArm(const std::unique_ptr<RobotConfig>& example_config, const Lookup& lookup, std::shared_ptr<arm::Arm> &arm_out , std::shared_ptr<arm::Gripper> &gripper_out)
{
    arm_out = arm::Arm::create(*example_config, lookup);
    while (!arm_out) {
        std::cerr << "Failed to create arm, retrying..." << std::endl;
        arm_out = arm::Arm::create(*example_config, lookup);
    }
    std::cout << "Arm connected." << std::endl;

    bool has_gripper = false;
    const auto user_data = example_config->getUserData();

    if (user_data.hasBool("has_gripper"))
        has_gripper = user_data.getBool("has_gripper");

    if (arm_out && has_gripper)
    {
        const std::string family = example_config->getFamilies()[0];
        auto gripper_group = lookup.getGroupFromNames({ family }, { "gripperSpool" });

        int tries = 3;
        while (!gripper_group && tries > 0)
        {
            std::cerr << "Looking for gripper module " << family << "/gripperSpool ...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            gripper_group = lookup.getGroupFromNames({ family }, { "gripperSpool" });
            --tries;
        }

        if (!gripper_group)
            return;

        double gripper_open_effort = -5; 
        double gripper_close_effort = 1;

        if(user_data.hasFloatList("gripper_open_effort"))
            gripper_open_effort = user_data.getFloat("gripper_open_effort");

	    if (user_data.hasFloatList("gripper_close_effort"))
            gripper_close_effort = user_data.getFloat("gripper_close_effort");

	    gripper_out = arm::Gripper::create(gripper_group, gripper_open_effort, gripper_close_effort);
            const std::string gripper_gains_file = example_config->getGains("gripper");

        if (!gripper_out && !gripper_out->loadGains(gripper_gains_file))
            throw std::runtime_error("Could not read or send gripper gains\n");

        gripper_out->open();
    }
}