#include "hexapod_parameters.hpp"
#include "xml_util/xml_helpers.hpp"

#include "xml_util/pugixml.hpp"

namespace hebi {

void HexapodParameters::resetToDefaults()
{
  stance_radius_ = 0.45f;
  default_body_height_ = 0.12f;
  min_z_ = -0.3f;
  max_z_ = -0.05f;
  max_r_ = 0.18f;
  step_threshold_rotate_ = 0.05f;
  step_threshold_shift_ = 0.02f;
  logging_enabled_ = true;
  low_log_frequency_hz_ = 5;
  high_log_frequency_hz_ = 200;
}

bool HexapodParameters::loadFromFile(std::string file)
{
  pugi::xml_document doc;
  auto res = doc.load_file(file.c_str());
  if (!res)
    return false;
  auto root = doc.child("hex_config");
  if (!root)
    return false;

  auto stance = root.child("stance");
  auto step_thresh = root.child("step_threshold");
  auto logging = root.child("logging");
  bool success =
    xml::trySetFloatParameter(stance.attribute("radius"), stance_radius_) &&
    xml::trySetFloatParameter(stance.attribute("body_height"), default_body_height_) &&
    xml::trySetFloatParameter(stance.attribute("min_foot_height"), min_z_) &&
    xml::trySetFloatParameter(stance.attribute("max_foot_height"), max_z_) &&
    xml::trySetFloatParameter(stance.attribute("max_shift_radius"), max_r_);
  success = success &&
    xml::trySetFloatParameter(step_thresh.attribute("rotate"), step_threshold_rotate_) &&
    xml::trySetFloatParameter(step_thresh.attribute("shift"), step_threshold_shift_);
  success = success &&
    xml::trySetBoolParameter(logging.attribute("enabled"), logging_enabled_) &&
    xml::trySetFloatParameter(logging.attribute("low_frequency_hz"), low_log_frequency_hz_);
    xml::trySetFloatParameter(logging.attribute("high_frequency_hz"), high_log_frequency_hz_);

  return success;
}

bool HexapodParameters::saveToFile(std::string file) const
{
  pugi::xml_document doc;

  auto root = doc.append_child("hex_config");
  root.append_attribute("name") = "config_root";

  auto stance = root.append_child("stance");
  auto step_thresh = root.append_child("step_threshold");
  auto logging = root.append_child("logging");
  stance.append_attribute("radius") = stance_radius_;
  stance.append_attribute("body_height") = default_body_height_;
  stance.append_attribute("min_foot_height") = min_z_;
  stance.append_attribute("max_foot_height") = max_z_;
  stance.append_attribute("max_shift_radius") = max_r_;
  step_thresh.append_attribute("rotate") = step_threshold_rotate_;
  step_thresh.append_attribute("shift") = step_threshold_shift_;
  logging.append_attribute("enabled") = logging_enabled_;
  logging.append_attribute("low_frequency_hz") = low_log_frequency_hz_;
  logging.append_attribute("high_frequency_hz") = high_log_frequency_hz_;

  return (doc.save_file(file.c_str()));
}

} // namespace hebi
