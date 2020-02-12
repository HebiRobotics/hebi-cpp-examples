#include "feedback.hpp"

#include <cmath>
#include <limits>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi {

Feedback::FloatField::FloatField(const HebiFeedbackRef& internal, HebiFeedbackFloatField field)
  : internal_(internal), field_(field) {}

bool Feedback::FloatField::has() const {
  return (floatGetter(internal_, field_, nullptr) == HebiStatusSuccess);
}

float Feedback::FloatField::get() const {
  float ret;
  if (floatGetter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::HighResAngleField::HighResAngleField(const HebiFeedbackRef& internal, HebiFeedbackHighResAngleField field)
  : internal_(internal), field_(field) {}

bool Feedback::HighResAngleField::has() const {
  return (highResAngleGetter(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

double Feedback::HighResAngleField::get() const {
  int64_t revolutions;
  float radian_offset;
  if (highResAngleGetter(internal_, field_, &revolutions, &radian_offset) != HebiStatusSuccess) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (static_cast<double>(revolutions) * 2.0 * M_PI + static_cast<double>(radian_offset));
}

void Feedback::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const {
  if (highResAngleGetter(internal_, field_, revolutions, radian_offset) != HebiStatusSuccess) {
    *revolutions = 0;
    *radian_offset = std::numeric_limits<float>::quiet_NaN();
  }
}

Feedback::NumberedFloatField::NumberedFloatField(const HebiFeedbackRef& internal, HebiFeedbackNumberedFloatField field)
  : internal_(internal), field_(field) {}

bool Feedback::NumberedFloatField::has(size_t fieldNumber) const {
  return (numberedFloatGetter(internal_, field_, fieldNumber, nullptr) == HebiStatusSuccess);
}

float Feedback::NumberedFloatField::get(size_t fieldNumber) const {
  float ret;
  if (numberedFloatGetter(internal_, field_, fieldNumber, &ret) != HebiStatusSuccess) {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::UInt64Field::UInt64Field(const HebiFeedbackRef& internal, HebiFeedbackUInt64Field field)
  : internal_(internal), field_(field) {}

bool Feedback::UInt64Field::has() const {
  return (uint64Getter(internal_, field_, nullptr) == HebiStatusSuccess);
}

uint64_t Feedback::UInt64Field::get() const {
  uint64_t ret;
  if (uint64Getter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret = 0;
  }
  return ret;
}

Feedback::Vector3fField::Vector3fField(const HebiFeedbackRef& internal, HebiFeedbackVector3fField field)
  : internal_(internal), field_(field) {}

bool Feedback::Vector3fField::has() const {
  return (vector3fGetter(internal_, field_, nullptr) == HebiStatusSuccess);
}

Vector3f Feedback::Vector3fField::get() const {
  HebiVector3f ret;
  if (vector3fGetter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret.x = std::numeric_limits<float>::quiet_NaN();
    ret.y = std::numeric_limits<float>::quiet_NaN();
    ret.z = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::QuaternionfField::QuaternionfField(const HebiFeedbackRef& internal, HebiFeedbackQuaternionfField field)
  : internal_(internal), field_(field) {}

bool Feedback::QuaternionfField::has() const {
  return (quaternionfGetter(internal_, field_, nullptr) == HebiStatusSuccess);
}

Quaternionf Feedback::QuaternionfField::get() const {
  HebiQuaternionf ret;
  if (quaternionfGetter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret.w = std::numeric_limits<float>::quiet_NaN();
    ret.x = std::numeric_limits<float>::quiet_NaN();
    ret.y = std::numeric_limits<float>::quiet_NaN();
    ret.z = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Feedback::IoBank::IoBank(const HebiFeedbackRef& internal, HebiFeedbackIoPinBank bank) : internal_(internal), bank_(bank) {}

bool Feedback::IoBank::hasInt(size_t pinNumber) const {
  return (intIoPinGetter(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

bool Feedback::IoBank::hasFloat(size_t pinNumber) const {
  return (floatIoPinGetter(internal_, bank_, pinNumber, nullptr) == HebiStatusSuccess);
}

int64_t Feedback::IoBank::getInt(size_t pinNumber) const {
  int64_t ret;
  intIoPinGetter(internal_, bank_, pinNumber, &ret);
  return ret;
}

float Feedback::IoBank::getFloat(size_t pinNumber) const {
  float ret;
  floatIoPinGetter(internal_, bank_, pinNumber, &ret);
  return ret;
}

Feedback::LedField::LedField(const HebiFeedbackRef& internal, HebiFeedbackLedField field)
  : internal_(internal), field_(field) {}

bool Feedback::LedField::hasColor() const {
  return ledGetter(internal_, field_, nullptr, nullptr, nullptr, nullptr) == HebiStatusSuccess;
}

Color Feedback::LedField::getColor() const {
  uint8_t r, g, b, a;
  if (ledGetter(internal_, field_, &r, &g, &b, &a) != HebiStatusSuccess) {
    r = 0;
    g = 0;
    b = 0;
    a = 0;
  }
  return Color(r, g, b, a);
}

Feedback::Feedback(HebiFeedbackPtr feedback)
  : internal_(feedback),
    io_(internal_ref_),
    actuator_(internal_ref_),
    mobile_(internal_ref_),
    imu_(internal_ref_),
    board_temperature_(internal_ref_, HebiFeedbackFloatBoardTemperature),
    processor_temperature_(internal_ref_, HebiFeedbackFloatProcessorTemperature),
    voltage_(internal_ref_, HebiFeedbackFloatVoltage),
    receive_time_us_(internal_ref_, HebiFeedbackUInt64ReceiveTime),
    transmit_time_us_(internal_ref_, HebiFeedbackUInt64TransmitTime),
    hardware_receive_time_us_(internal_ref_, HebiFeedbackUInt64HardwareReceiveTime),
    hardware_transmit_time_us_(internal_ref_, HebiFeedbackUInt64HardwareTransmitTime),
    sender_id_(internal_ref_, HebiFeedbackUInt64SenderId),
    debug_(internal_ref_, HebiFeedbackNumberedFloatDebug),
    led_(internal_ref_, HebiFeedbackLedLed) {
  hebiFeedbackGetReference(internal_, &internal_ref_);
}

Feedback::Feedback(Feedback&& other)
  : internal_(other.internal_),
    io_(internal_ref_),
    actuator_(internal_ref_),
    mobile_(internal_ref_),
    imu_(internal_ref_),
    board_temperature_(internal_ref_, HebiFeedbackFloatBoardTemperature),
    processor_temperature_(internal_ref_, HebiFeedbackFloatProcessorTemperature),
    voltage_(internal_ref_, HebiFeedbackFloatVoltage),
    receive_time_us_(internal_ref_, HebiFeedbackUInt64ReceiveTime),
    transmit_time_us_(internal_ref_, HebiFeedbackUInt64TransmitTime),
    hardware_receive_time_us_(internal_ref_, HebiFeedbackUInt64HardwareReceiveTime),
    hardware_transmit_time_us_(internal_ref_, HebiFeedbackUInt64HardwareTransmitTime),
    sender_id_(internal_ref_, HebiFeedbackUInt64SenderId),
    debug_(internal_ref_, HebiFeedbackNumberedFloatDebug),
    led_(internal_ref_, HebiFeedbackLedLed) {
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
  hebiFeedbackGetReference(internal_, &internal_ref_);
}

} // namespace hebi
