#pragma once

// HEBI C++ API files:
#include "arm/arm.hpp"
#include "lookup.hpp"
#include "util/mobile_io.hpp"
#include "color.hpp"


using namespace hebi;

// Set the message on the MobileIO device and print instructions
void setMobileIOInstructions(util::MobileIO& mobile_io, const std::string& message, Color color = Color{ 0,0,0 });

// Setup the arm and gripper based on the configuration file
void setupArm(const RobotConfig& example_config, const Lookup& lookup, std::shared_ptr<arm::Arm>& arm_out, std::shared_ptr<arm::Gripper>& gripper_out);
