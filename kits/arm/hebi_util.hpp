// hebi_util.hpp

#ifndef HEBI_UTIL_HPP
#define HEBI_UTIL_HPP

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"

#include <stdexcept>
#include <filesystem>
#include <iostream>

#include "robot_config.hpp"
#include "arm/arm.hpp"
#include "util/mobile_io.hpp"
#include <memory>
#include <string>

// hebi_util stores helper functions that may be rolled into the API under hebi::util

// Function to create Mobile IO from the config
std::unique_ptr<hebi::util::MobileIO> createMobileIOFromConfig(const hebi::RobotConfig& example_config);

// TODO: Function to create gripper from the config
// std::unique_ptr<hebi::experimental::arm::EndEffectorBase> createGripperFromConfig(const hebi::RobotConfig& example_config, const std::string& config_file_path);

#endif // HEBI_UTIL_HPP
