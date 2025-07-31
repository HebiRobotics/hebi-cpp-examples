// test commit

// HEBI C++ API files:
#include "arm/arm.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"
#include "util/mobile_io.hpp"
#include "util/vector_utils.h"
#include "rosie_demo_utils.hpp"
#include "kits/arms/ar_control_sm.hpp"
#include "kits/bases/omni_base_final.cpp"
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <ctime>
#include <cstdlib>

// Common includes
#include <iostream>

using namespace hebi;
double ar_scaling = 1.0;

// RosieControl controls the base of the robot and builds a trajectory for the base movement
class RosieControl {
public:
    RosieControl(OmniBase& base)
        : base_(base), running_(true) {}

    void update(const double t_now, const ChassisVelocity* base_inputs)
    {
        if (!base_inputs)
            return;

		base_.update(t_now, base_inputs->x_, base_inputs->y_, base_inputs->rz_);
    }

    void send() const {
        base_.send();
    }

    bool running() const { return running_; }

    void stop() {
        running_ = false;
        base_.stop();
    }

private:
    OmniBase& base_;
    bool running_{ false };
};

std::function<void(ArmMobileIOControl&, ArmControlState)> updateMobileIO(util::MobileIO& mio) {
    return [&mio](ArmMobileIOControl& controller, ArmControlState new_state) {
        if (controller.state_ == new_state)
            return;

        switch (new_state) {

        case ArmControlState::HOMING:
            controller.setArmLedColor(Color{ 255, 0, 255 }); //magenta
            setMobileIOInstructions(mio, "Robot Homing Sequence\nPlease wait...", Color{ 255, 0, 255 }); //magenta
            break;

        case ArmControlState::TELEOP:
            controller.setArmLedColor(Color{ 0, 255, 0 }); //green
            setMobileIOInstructions(mio, "Robot Ready to Control", Color{ 0, 255, 0 }); //green
            break;

        case ArmControlState::DISCONNECTED:
            controller.setArmLedColor(Color{ 0, 0, 255 }); //blue
			setMobileIOInstructions(mio, "Robot Disconnected", Color{ 0, 0, 255 }); //blue
            break;

        case ArmControlState::EXIT:
            std::cout << "TRANSITIONING TO EXIT" << std::endl;
            controller.setArmLedColor(Color{ 255, 0, 0 }); //red
            mio.resetUI();
            setMobileIOInstructions(mio, "Demo Stopped.", Color{ 255, 0, 0 }); //red
            break;

        default:
            break;
        }
    };
}

// Sets up mobileIO interface.
//
//Return a function that parses mobileIO feedback into the format
//expected by the Demo

std::function<bool(ArmControlState, ChassisVelocity&, ArmMobileIOInputs&)> setupMobileIO(util::MobileIO& mio) {

    const int reset_pose_btn = 1;
    const int arm_lock = 3;
    const int gripper_close = 4;
    const int quit_demo_btn = 8;
    
    const int side_joy = 1;  // Left Pad Left/Right
    const int forward_joy = 2; // Left Pad Up/Down
    const int ar_xyz_scale_slider = 4;
    const int turn_joy = 7;  // Right Pad Left/Right
	const int rotate_joy = 8; // Right Pad Up/Down

    mio.resetUI();

    mio.setButtonLabel(reset_pose_btn, "Home \u27F2", false);
    mio.setButtonLabel(quit_demo_btn, "Quit \u274C", false);

    mio.setAxisLabel(side_joy, "", false);
    mio.setAxisLabel(forward_joy, "Translate", false);
    mio.setAxisLabel(ar_xyz_scale_slider, "XYZ\nScale", false);
    mio.setAxisLabel(turn_joy, "", false);
    mio.setAxisLabel(rotate_joy, "Rotate", false);

    for (int i = 0; i < 8; i++)
      mio.setAxisSnap(i + 1, (i == 3) ? NAN : 0.0);

    mio.setAxisValue(ar_xyz_scale_slider, ar_scaling);
    mio.setLedColor(255, 255, 0);

    mio.setButtonLabel(arm_lock, "Arm \U0001F512", false);
    mio.setButtonLabel(gripper_close, "Gripper", false);
    mio.setButtonMode(arm_lock, util::MobileIO::ButtonMode::Toggle);
    mio.setButtonMode(gripper_close, util::MobileIO::ButtonMode::Toggle);

	// Lambda function that will be called to parse the mobile IO feedback
    auto parseMobileIOFeedback = [&](ArmControlState curr_state, ChassisVelocity& chassis_velocity_out, ArmMobileIOInputs& arm_inputs_out) {
        if (!mio.update(0.0))
            return false;

        if (mio.getButton(quit_demo_btn))
            return true;

        if (mio.getButton(reset_pose_btn))
        {
            arm_inputs_out.home = true;
            return false;
        }

        chassis_velocity_out = ChassisVelocity{
            pow(mio.getAxis(forward_joy), 3),
            pow(-mio.getAxis(side_joy), 3),
            pow(-mio.getAxis(turn_joy), 3) * 2 };

        auto wxyz = mio.getArOrientation();
        Eigen::Quaterniond q(wxyz.getW(), wxyz.getX(), wxyz.getY(), wxyz.getZ());
        Eigen::Matrix3d rotation = q.toRotationMatrix();

        if (!rotation.allFinite()) {
            std::cerr << "Error getting orientation as matrix: " << wxyz.getW() << ", " << wxyz.getX() << ", "
                << wxyz.getY() << ", " << wxyz.getZ() << "\n";
            rotation = Eigen::Matrix3d::Identity();
        }

        if (mio.getButtonDiff(arm_lock) != util::MobileIO::ButtonState::Unchanged)
        {
            if (!mio.getButton(arm_lock))
                mio.setButtonLabel(arm_lock, "Arm \U0001F512", false);
            else
                mio.setButtonLabel(arm_lock, "Arm \U0001F513", false);
        }

        bool locked = mio.getButton(arm_lock);
        if (locked) {
            mio.setAxisValue(ar_xyz_scale_slider, (2 * ar_scaling) - 1.0);
        }
        else {
            ar_scaling = (mio.getAxis(ar_xyz_scale_slider) + 1.0) / 2.0;
            if (ar_scaling < 0.1)
                ar_scaling = 0.0;
        }

        arm_inputs_out = ArmMobileIOInputs(
            mio.getArPosition(),
            rotation,
            ar_scaling,
            (mio.getButtonDiff(arm_lock) != util::MobileIO::ButtonState::Unchanged),
            !locked,
            mio.getButton(gripper_close)
        );

        return false;
	};

	return parseMobileIOFeedback;
}

int main(int argc, char** argv) {


    //////////////////////////
    ///// Config Setup ///////
    //////////////////////////

    // Config file path
    std::string example_config_path;
    if (argc == 1) {
        example_config_path = "config/rosie-r.cfg.yaml";
    }
    else if (argc == 2) {
        example_config_path = "config/" + std::string(argv[1]);
    }
    else {
        std::cout << "Run ./rosie_demo <optional config file name>\n";
        return 1;
    }

    // Load the config
    std::vector<std::string> errors;
    const auto example_config = RobotConfig::loadConfig(example_config_path, errors);
    for (const auto& error : errors) {
        std::cerr << error << std::endl;
    }
    if (!example_config) {
        std::cerr << "Failed to load configuration from: " << example_config_path << std::endl;
        return 1;
    }

    Lookup lookup;
    const std::string family = example_config->getFamilies()[0];


    //////////////////////////
    //// MobileIO Setup //////
    //////////////////////////

    std::unique_ptr<util::MobileIO> mobile_io = util::MobileIO::create(family, "mobileIO", lookup);
    int mobileio_tries = 5;

    while (!mobile_io && mobileio_tries > 0) {
        std::cout << "Waiting for mobileIO device to come online..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        mobile_io = util::MobileIO::create(family, "mobileIO", lookup);
        --mobileio_tries;
    }

    if (!mobile_io)
    {
        std::cout << "Couldn't find mobile IO device!\n";
        return 1;
    }

    std::cout << "MobileIO device successfully connected\n";
    std::shared_ptr<util::MobileIO> mobile_io_shared = std::move(mobile_io);
    auto parse_mobile_feedback = setupMobileIO(*mobile_io_shared);


    //////////////////////////
    ////// Base Setup ////////
    //////////////////////////

	int base_tries = 5;
    const std::vector<std::string> wheel_names = { "W1", "W2", "W3" };
    auto wheel_group = lookup.getGroupFromNames({ family }, wheel_names);

    if (!wheel_group && base_tries > 0) {
        std::cout << "Retrying to find wheel modules for the base..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto wheel_group = lookup.getGroupFromNames({ family }, wheel_names);
        --base_tries;
    }

    if (!wheel_group)
    {
        std::cout << "Couldn't find wheel modules device!\n";
        return 1;
    }

    std::cout << "Base wheel modules connected successfully\n";
 
    OmniBase base = OmniBase(wheel_group);
    if (!base.setGains())
        return 1;
    auto base_control = RosieControl(base);


    //////////////////////////
    ///// Arm Setup //////////
    //////////////////////////

    // Create the arm and gripper object from the configuration
    std::shared_ptr<arm::Arm> arm;
    std::shared_ptr<arm::Gripper> gripper;
    setupArm(*example_config, lookup, arm, gripper);

    while (!arm->update()) {
        std::cout << "Waiting for feedback from arm..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    const auto user_data = example_config->getUserData();
    const double soft_start_time = user_data.getFloat("homing_duration");
    const Eigen::Vector3d xyz_scale = util::stdToEigenXd(user_data.getFloatList("xyz_scale"));
    const double delay_time = user_data.getFloat("delay_time");

    std::unique_ptr<ArmMobileIOControl> arm_control (new ArmMobileIOControl(arm, gripper, soft_start_time, delay_time, xyz_scale));
    arm_control->transition_handlers_.push_back(updateMobileIO(*mobile_io_shared));


    ///////////////////////////
    //// Main Control Loop ////
    //////////////////////////

    bool enable_logging = true;
    if (enable_logging) {
        base.group_->startLog("./logs");
        arm->group().startLog("./logs");
    }

    ChassisVelocity base_inputs;
    ArmMobileIOInputs arm_inputs;
    auto start = std::chrono::system_clock::now();

	// Constantly update the base and arm controls
    while (base_control.running() && (!arm_control || arm_control->running())) {

        std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
        bool quit = parse_mobile_feedback(arm_control->state_, base_inputs, arm_inputs);
        base_control.update(t.count(), &base_inputs);

        if (arm_control)
            arm_control->update(t.count(), &arm_inputs);

        if (quit) {
            base_control.stop();
            if (arm_control)
                arm_control->transition_to(t.count(), ArmControlState::EXIT);
        }

        base_control.send();
        if (arm_control)
            arm_control->send();
    }

    if (enable_logging) {
        base.group_->stopLog();
        if (arm)
            arm->group().stopLog();
    }

    return 0;
}
