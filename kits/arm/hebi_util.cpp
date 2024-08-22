// hebi_util.cpp

#include "group_command.hpp"
#include "group_feedback.hpp"
#include "robot_model.hpp"
#include "arm/arm.hpp"
#include "util/file.hpp"

#include <stdexcept>
#include <filesystem>
#include <iostream>

#include "hebi_util.hpp"

std::unique_ptr<hebi::util::MobileIO> createMobileIOFromConfig(const hebi::RobotConfig& config) {
    std::map<std::string, std::string> mobile_io_dict;
    std::vector<std::string> errors;

    auto parent_dir_absolute = config.getLocation();

    // Lambda function to check file paths
    auto check_file = [&errors, &parent_dir_absolute](const std::string& type, const std::string& relative_filename) {
        // Ensure file is relative:
        if (hebi::util::file::File(relative_filename).isAbsolute()) {
            errors.push_back("'" + type + "' exists but provides an absolute path.");
            return std::string();
        }
        // Ensure file exists
        hebi::util::file::File absolute_filename(parent_dir_absolute);
        absolute_filename.append(relative_filename);
        if (!absolute_filename.exists()) {
            errors.push_back("'" + type + "' exists but does not point to a valid file.");
            return std::string();
        }
        // Return success!
        return absolute_filename.getAbsolutePath();
    };

    // Validate the mobile_io configuration
    if (config.getUserData().strings_.count("mobile_io_family") &&
        config.getUserData().strings_.count("mobile_io_name") &&
        config.getUserData().strings_.count("mobile_io_layout")) {

        // Check that all required fields are present and are strings
        if (config.getUserData().getString("mobile_io_family").empty() ||
            config.getUserData().getString("mobile_io_name").empty()) {
            errors.push_back("HEBI config \"user_data\"'s \"mobile_io_...\" fields must be non-empty strings.");
        }

        // Populate the dictionary
        mobile_io_dict["family"] = config.getUserData().getString("mobile_io_family");
        mobile_io_dict["name"] = config.getUserData().getString("mobile_io_name");

        // Use check_file to validate and convert layout to absolute path
        mobile_io_dict["layout"] = check_file("mobile_io_layout", config.getUserData().getString("mobile_io_layout"));
        if (mobile_io_dict["layout"].empty()) {
            errors.push_back("HEBI config \"user_data\"'s \"mobile_io_layout\" file is invalid.");
        }
    } else {
        errors.push_back("HEBI config \"user_data\" must contain the keys: \"mobile_family\", \"mobile_name\", and \"mobile_layout\".");
    }

    // If there are errors, print them and throw an exception
    if (!errors.empty()) {

        for (const auto& error : errors) {
            std::cerr << "Error: " << error << std::endl;
        }
        throw std::runtime_error("Failed to create MobileIO due to configuration errors.");
    }

    // Create Mobile IO based on validated config
    std::unique_ptr<hebi::util::MobileIO> mobile_io = hebi::util::MobileIO::create(mobile_io_dict["family"], mobile_io_dict["name"]);

    if(mobile_io != nullptr)
    {
        mobile_io->sendLayoutFile(mobile_io_dict["layout"]);
    }
    return mobile_io;
}