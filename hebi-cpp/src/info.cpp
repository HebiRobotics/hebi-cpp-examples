#include "info.hpp"

#include <limits>

namespace hebi {

Info::FloatField::FloatField(const HebiInfoRef& internal, HebiInfoFloatField field) : internal_(internal), field_(field) {}

bool Info::FloatField::has() const { return (floatGetter(internal_, field_, nullptr) == HebiStatusSuccess); }

float Info::FloatField::get() const {
  float ret;
  if (floatGetter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Info::HighResAngleField::HighResAngleField(const HebiInfoRef& internal, HebiInfoHighResAngleField field)
  : internal_(internal), field_(field) {}

bool Info::HighResAngleField::has() const {
  return (highResAngleGetter(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

double Info::HighResAngleField::get() const {
  int64_t revolutions;
  float radian_offset;
  if (highResAngleGetter(internal_, field_, &revolutions, &radian_offset) != HebiStatusSuccess) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (static_cast<double>(revolutions) * 2.0 * M_PI + static_cast<double>(radian_offset));
}

void Info::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const {
  if (highResAngleGetter(internal_, field_, revolutions, radian_offset) != HebiStatusSuccess) {
    *revolutions = 0;
    *radian_offset = std::numeric_limits<float>::quiet_NaN();
  }
}

Info::BoolField::BoolField(const HebiInfoRef& internal, HebiInfoBoolField field) : internal_(internal), field_(field) {}

bool Info::BoolField::has() const { return (boolGetter(internal_, field_, nullptr) == HebiStatusSuccess); }

bool Info::BoolField::get() const {
  bool ret{};
  boolGetter(internal_, field_, &ret);
  return static_cast<bool>(ret);
}

Info::StringField::StringField(HebiInfoPtr internal, HebiInfoStringField field) : internal_(internal), field_(field) {}

bool Info::StringField::has() const {
  return (hebiInfoGetString(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

std::string Info::StringField::get() const {
  // Get the size first
  size_t length;
  if (hebiInfoGetString(internal_, field_, nullptr, &length) != HebiStatusSuccess) {
    // String field doesn't exist -- return an empty string
    return "";
  }
  auto buffer = new char[length];
  hebiInfoGetString(internal_, field_, buffer, &length);
  std::string tmp(buffer, length - 1);
  delete[] buffer;
  return tmp;
}

Info::FlagField::FlagField(const HebiInfoRef& internal, HebiInfoFlagField field) : internal_(internal), field_(field) {}

bool Info::FlagField::has() const { return (flagGetter(internal_, field_) == 1); }

Info::LedField::LedField(const HebiInfoRef& internal, HebiInfoLedField field) : internal_(internal), field_(field) {}

bool Info::LedField::hasColor() const {
  return ledGetter(internal_, field_, nullptr, nullptr, nullptr, nullptr) == HebiStatusSuccess;
}

Color Info::LedField::getColor() const {
  uint8_t r, g, b, a;
  if (ledGetter(internal_, field_, &r, &g, &b, &a) != HebiStatusSuccess) {
    r = 0;
    g = 0;
    b = 0;
    a = 0;
  }
  return Color(r, g, b, a);
}

Info::Info(HebiInfoPtr info)
  : internal_(info),
    settings_(internal_, internal_ref_),
    actuator_(internal_ref_),
    serial_(internal_, HebiInfoStringSerial),
    led_(internal_ref_, HebiInfoLedLed) {
  hebiInfoGetReference(internal_, &internal_ref_);
}

Info::Info(Info&& other)
  : internal_(other.internal_),
    settings_(internal_, internal_ref_),
    actuator_(internal_ref_),
    serial_(internal_, HebiInfoStringSerial),
    led_(internal_ref_, HebiInfoLedLed) {
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
  hebiInfoGetReference(internal_, &internal_ref_);
}

} // namespace hebi
