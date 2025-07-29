#include "ar_control_sm.hpp"


ArmMobileIOInputs::ArmMobileIOInputs(
    const hebi::Vector3f& pos,
    const Eigen::Matrix3d& rot,
    const double scaling,
    bool lockToggle,
    bool isLocked,
    bool gripperClosed,
    bool isHome) :
    phone_pos(pos),
    phone_rot(rot),
    ar_scaling(scaling),
    lock_toggle(lockToggle),
    locked(isLocked),
    gripper_closed(gripperClosed),
    home(isHome) {}


ArmMobileIOControl::ArmMobileIOControl(const std::shared_ptr<arm::Arm> arm, const std::shared_ptr<arm::Gripper> gripper, const double homing_time, const double traj_duration, const Eigen::Vector3d xyz_scale) : arm_(arm), gripper_(gripper), homing_time_(homing_time), traj_duration_(traj_duration), xyz_scale_(xyz_scale) {
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

void ArmMobileIOControl::send() const {
    arm_->send();
    if (gripper_)
        gripper_->send();
}

void ArmMobileIOControl::home(const double duration) const {
    arm_->setGoal(arm::Goal::createFromPosition(duration, arm_home_));
}

void ArmMobileIOControl::transition_to(const double t_now, const ArmControlState& new_state) {
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

arm::Goal ArmMobileIOControl::compute_arm_goal(const ArmMobileIOInputs& arm_input) {
    Eigen::Vector3d phone_offset(
        arm_input.phone_pos.getX() - phone_xyz_home_.getX(),
        arm_input.phone_pos.getY() - phone_xyz_home_.getY(),
        arm_input.phone_pos.getZ() - phone_xyz_home_.getZ());

    auto rot_mat = phone_rot_home_;
    auto arm_xyz_target = last_locked_xyz_ + arm_input.ar_scaling * xyz_scale_.cwiseProduct(rot_mat.transpose() * phone_offset);
    Eigen::Matrix3d arm_rot_target = rot_mat.transpose() * arm_input.phone_rot * last_locked_rot_;


    // if ar scaling is 0, move the home AR pose to current pose this keeps the arm from driving to some weird offset when scaling is turned back up by the user in the future
    if (arm_input.ar_scaling == 0.0)
        phone_xyz_home_ = arm_input.phone_pos;
    auto joint_target = arm_->solveIK(last_locked_seed_, arm_xyz_target, arm_rot_target);
    arm::Goal goal = arm::Goal::createFromPosition(traj_duration_, joint_target);

    return goal;
}

void ArmMobileIOControl::update(const double t_now, const ArmMobileIOInputs* arm_input) {
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

    // Reset the timeout
    mobile_last_fbk_t_ = t_now;


    // Transition to teleop if mobileIO is reconnected
    auto last_pos = arm_->lastFeedback().getPosition();

    switch (state_) {

    case ArmControlState::DISCONNECTED:
        mobile_last_fbk_t_ = t_now;
        std::cout << namespace_ << " Controller reconnected, demo continued\n";

        /// Lock arm when reconnected
        locked_ = true;
        transition_to(t_now, ArmControlState::TELEOP);
        break;

    // After startup, transition to homing
    case ArmControlState::STARTUP:
        transition_to(t_now, ArmControlState::HOMING);
        break;

    // If homing is complete, transition to teleop
    case ArmControlState::HOMING:
        if (arm_->atGoal()) {
            phone_xyz_home_ = arm_input->phone_pos;
            phone_rot_home_ = arm_input->phone_rot;

            last_locked_seed_ = last_pos;
            arm_->FK(last_pos, last_locked_xyz_, last_locked_rot_);
            transition_to(t_now, ArmControlState::TELEOP);
        }
        break;

    // In teleop mode, update the arm based on mobile IO inputs
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
                std::cout << "Gripper Close" << std::endl;
                gripper_->close();
            }
            else if (!arm_input->gripper_closed && gripper_closed) {
                std::cout << "Gripper Open" << std::endl;
                gripper_->open();
            }
        }
        break;
    }
}
    
void ArmMobileIOControl::setArmLedColor(const Color& color) {
    auto group_size = this->arm_->pendingCommand().size();
    for (int i = 0; i < group_size; i++)
        this->arm_->pendingCommand()[i].led().set(color);
}

void ArmMobileIOControl::stop() {
    auto now = std::chrono::system_clock::now();
    std::time_t time_now = std::chrono::system_clock::to_time_t(now);
    transition_to(time_now, ArmControlState::EXIT);
}