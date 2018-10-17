#pragma once

#include "pugixml.hpp"

namespace hebi {

class XMLHelpers
{
public:
  static bool trySetFloatParameter(pugi::xml_attribute&& attr, float& param);
  static bool trySetBoolParameter(pugi::xml_attribute&& attr, bool& param);
  static bool trySetStringParameter(pugi::xml_attribute&& attr, std::string& param);
private:
  XMLHelpers() = delete;
};

} // namespace hebi
