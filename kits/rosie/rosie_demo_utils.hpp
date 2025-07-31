#pragma once

// HEBI C++ API files:
#include "arm/arm.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "util/mobile_io.hpp"
#include "group.hpp"
#include "color.hpp"
#include <thread>
#include <chrono>

// Common includes
#include <iostream>

using namespace hebi;

// Set the message on the MobileIO device and print instructions
void setMobileIOInstructions(util::MobileIO& mobile_io, const std::string& message, const Color& color = Color{ 0,0,0 });

// Setup the arm and gripper based on the configuration file
void setupArm(const RobotConfig& example_config, const Lookup& lookup, std::shared_ptr<arm::Arm>& arm_out, std::shared_ptr<arm::Gripper>& gripper_out);
