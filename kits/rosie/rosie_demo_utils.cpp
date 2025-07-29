#include "rosie_demo_utils.hpp"


void setMobileIOInstructions(util::MobileIO& mobile_io, const std::string& message, const Color& color) {

    mobile_io.setLedColor(color.getRed(), color.getGreen(), color.getBlue(), false);
    mobile_io.clearText(false);
    mobile_io.appendText(message, false);

    std::cout << message << std::endl;
}

void setupArm(const RobotConfig& example_config, const Lookup& lookup, std::shared_ptr<arm::Arm>& arm_out , std::shared_ptr<arm::Gripper>& gripper_out)
{
    int arm_tries = 5;
    auto arm = arm::Arm::create(example_config, lookup);

    while (!arm && arm_tries > 0) {
        std::cout << "Failed to create arm, retrying..." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
        arm = arm::Arm::create(example_config, lookup);
        arm_tries--;
    }

    if (!arm)
    {
        std::cout << "Failed to create arm" << std::endl ;
        return;
    }

    std::cout << "Arm connected." << std::endl;
	arm_out = std::move(arm);

    bool has_gripper = false;
    const auto user_data = example_config.getUserData();

    if (user_data.hasBool("has_gripper"))
        has_gripper = user_data.getBool("has_gripper");
    
	// Setup gripper parameters if specified in the config
    if (arm_out && has_gripper)
    {
        const std::string family = example_config.getFamilies()[0];
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
            const std::string gripper_gains_file = example_config.getGains("gripper");

        if (!gripper_out || !gripper_out->loadGains(gripper_gains_file)) {
            std::cout << "Could not read or send gripper gains" << std::endl;
            return;
        }

        gripper_out->open();
    }
}