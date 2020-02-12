#include "command.hpp"

#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi {

Command::FloatField::FloatField(HebiCommandRef& internal, HebiCommandFloatField field)
  : internal_(internal), field_(field) {}

bool Command::FloatField::has() const { return (floatGetter(internal_, field_, nullptr) == HebiStatusSuccess); }

float Command::FloatField::get() const {
  float ret;
  if (floatGetter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

void Command::FloatField::set(float value) { hebiCommandSetFloat(internal_, field_, &value); }

void Command::FloatField::clear() { hebiCommandSetFloat(internal_, field_, nullptr); }

Command::HighResAngleField::HighResAngleField(HebiCommandRef& internal, HebiCommandHighResAngleField field)
  : internal_(internal), field_(field) {}

bool Command::HighResAngleField::has() const {
  return (highResAngleGetter(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

double Command::HighResAngleField::get() const {
  int64_t revolutions;
  float radian_offset;
  if (highResAngleGetter(internal_, field_, &revolutions, &radian_offset) != HebiStatusSuccess) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (static_cast<double>(revolutions) * 2.0 * M_PI + static_cast<double>(radian_offset));
}

void Command::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const {
  if (highResAngleGetter(internal_, field_, revolutions, radian_offset) != HebiStatusSuccess) {
    *revolutions = 0;
    *radian_offset = std::numeric_limits<float>::quiet_NaN();
  }
}

void Command::HighResAngleField::set(double radians) {
  double revolutions_raw = radians / 2.0 / M_PI;
  double revolutions_int_d;
  double radian_offset_d = std::modf(revolutions_raw, &revolutions_int_d);
  radian_offset_d = radian_offset_d * 2.0 * M_PI;

  int64_t revolutions_int = std::isnan(revolutions_int_d) ? 0 : static_cast<int64_t>(revolutions_int_d);
  float radian_offset = static_cast<float>(radian_offset_d);
  hebiCommandSetHighResAngle(internal_, field_, &revolutions_int, &radian_offset);
}

void Command::HighResAngleField::set(int64_t revolutions, float radian_offset) {
  hebiCommandSetHighResAngle(internal_, field_, &revolutions, &radian_offset);
}

void Command::HighResAngleField::clear() { hebiCommandSetHighResAngle(internal_, field_, nullptr, nullptr); }

Command::NumberedFloatField::NumberedFloatField(HebiCommandRef& internal, HebiCommandNumberedFloatField field)
  : internal_(internal), field_(field) {}

bool Command::NumberedFloatField::has(size_t fieldNumber) const {
  return (numberedFloatGetter(internal_, field_, fieldNumber, nullptr) == HebiStatusSuccess);
}

float Command::NumberedFloatField::get(size_t fieldNumber) const {
  float ret;
  if (numberedFloatGetter(internal_, field_, fieldNumber, &ret) != HebiStatusSuccess) {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

void Command::NumberedFloatField::set(size_t fieldNumber, float value) {
  hebiCommandSetNumberedFloat(internal_, field_, fieldNumber, &value);
}

void Command::NumberedFloatField::clear(size_t fieldNumber) {
  hebiCommandSetNumberedFloat(internal_, field_, fieldNumber, nullptr);
}

Command::BoolField::BoolField(HebiCommandRef& internal, HebiCommandBoolField field)
  : internal_(internal), field_(field) {}

bool Command::BoolField::has() const { return (boolGetter(internal_, field_, nullptr) == HebiStatusSuccess); }

bool Command::BoolField::get() const {
  bool ret{};
  boolGetter(internal_, field_, &ret);
  return static_cast<bool>(ret);
}

void Command::BoolField::set(bool value) {
  auto val = static_cast<int>(value);
  hebiCommandSetBool(internal_, field_, &val);
}

void Command::BoolField::clear() { hebiCommandSetBool(internal_, field_, nullptr); }

Command::StringField::StringField(HebiCommandPtr internal, HebiCommandStringField field)
  : internal_(internal), field_(field) {}

bool Command::StringField::has() const {
  return (hebiCommandGetString(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

std::string Command::StringField::get() const {
  // Get the size first
  size_t length;
  if (hebiCommandGetString(internal_, field_, nullptr, &length) != HebiStatusSuccess) {
    // String field doesn't exist -- return an empty string
    return "";
  }
  auto buffer = new char[length];
  hebiCommandGetString(internal_, field_, buffer, &length);
  std::string tmp(buffer, length - 1);
  delete[] buffer;
  return tmp;
}

void Command::StringField::set(const std::string& value) {
  const char* buffer = value.c_str();
  size_t length = value.size();
  hebiCommandSetString(internal_, field_, buffer, &length);
}

void Command::StringField::clear() { hebiCommandSetString(internal_, field_, nullptr, nullptr); }

Command::FlagField::FlagField(HebiCommandRef& internal, HebiCommandFlagField field)
  : internal_(internal), field_(field) {}

bool Command::FlagField::has() const { return flagGetter(internal_, field_) == 1; }

void Command::FlagField::set() { hebiCommandSetFlag(internal_, field_, 1); }

void Command::FlagField::clear() { hebiCommandSetFlag(internal_, field_, 0); }

Command::IoBank::IoBank(HebiCommandRef& internal, HebiCommandIoPinBank bank) : internal_(internal), bank_(bank) {}

bool Command::IoBank::hasInt(size_t pinNumber) const {
  return (intIoPinGetter(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

bool Command::IoBank::hasFloat(size_t pinNumber) const {
  return (floatIoPinGetter(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

int64_t Command::IoBank::getInt(size_t pinNumber) const {
  int64_t ret;
  intIoPinGetter(internal_, bank_, pinNumber, &ret);
  return ret;
}

float Command::IoBank::getFloat(size_t pinNumber) const {
  float ret;
  floatIoPinGetter(internal_, bank_, pinNumber, &ret);
  return ret;
}

void Command::IoBank::setInt(size_t pinNumber, int64_t value) {
  hebiCommandSetIoPinInt(internal_, bank_, pinNumber, &value);
}

void Command::IoBank::setFloat(size_t pinNumber, float value) {
  hebiCommandSetIoPinFloat(internal_, bank_, pinNumber, &value);
}

void Command::IoBank::clear(size_t pinNumber) {
  hebiCommandSetIoPinInt(internal_, bank_, pinNumber, nullptr);
  hebiCommandSetIoPinFloat(internal_, bank_, pinNumber, nullptr);
}

Command::LedField::LedField(HebiCommandRef& internal, HebiCommandLedField field) : internal_(internal), field_(field) {}

bool Command::LedField::has() const {
  return ledGetter(internal_, field_, nullptr, nullptr, nullptr, nullptr) == HebiStatusSuccess;
}

Color Command::LedField::get() const {
  uint8_t r, g, b, a;
  if (ledGetter(internal_, field_, &r, &g, &b, &a) != HebiStatusSuccess) {
    r = 0;
    g = 0;
    b = 0;
    a = 0;
  }
  return Color(r, g, b, a);
}

void Command::LedField::set(const Color& new_color) {
  hebiCommandSetLed(internal_, field_, &new_color); 
}

void Command::LedField::clear() {
  hebiCommandSetLed(internal_, field_, nullptr);
}

Command::Command(HebiCommandPtr command)
  : internal_(command),
    io_(internal_ref_),
    settings_(internal_, internal_ref_),
    actuator_(internal_ref_),
    debug_(internal_ref_, HebiCommandNumberedFloatDebug),
    append_log_(internal_, HebiCommandStringAppendLog),
    reset_(internal_ref_, HebiCommandFlagReset),
    boot_(internal_ref_, HebiCommandFlagBoot),
    stop_boot_(internal_ref_, HebiCommandFlagStopBoot),
    clear_log_(internal_ref_, HebiCommandFlagClearLog),
    led_(internal_ref_, HebiCommandLedLed) {
  hebiCommandGetReference(internal_, &internal_ref_);
}

Command::Command(Command&& other)
  : internal_(other.internal_),
    io_(internal_ref_),
    settings_(internal_, internal_ref_),
    actuator_(internal_ref_),
    debug_(internal_ref_, HebiCommandNumberedFloatDebug),
    append_log_(internal_, HebiCommandStringAppendLog),
    reset_(internal_ref_, HebiCommandFlagReset),
    boot_(internal_ref_, HebiCommandFlagBoot),
    stop_boot_(internal_ref_, HebiCommandFlagStopBoot),
    clear_log_(internal_ref_, HebiCommandFlagClearLog),
    led_(internal_ref_, HebiCommandLedLed) {
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
  hebiCommandGetReference(internal_, &internal_ref_);
}

} // namespace hebi