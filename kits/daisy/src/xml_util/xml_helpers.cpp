#include "xml_helpers.hpp"

namespace hebi {

bool XMLHelpers::trySetFloatParameter(pugi::xml_attribute&& attr, float& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_float();
    return true;
  }
  return false;
}

bool XMLHelpers::trySetBoolParameter(pugi::xml_attribute&& attr, bool& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_bool();
    return true;
  }
  return false;
}

bool XMLHelpers::trySetStringParameter(pugi::xml_attribute&& attr, std::string& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_string();
    return true;
  }
  return false;
}

} // namespace hebi
