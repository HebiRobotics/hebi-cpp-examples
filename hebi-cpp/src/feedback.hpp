#pragma once

#include "hebi.h"

#include "color.hpp"
#include "quaternion_f.hpp"
#include "message_helpers.hpp"
#include "util.hpp"
#include "vector_3_f.hpp"

namespace hebi {

/// \brief Feedback objects have various fields representing feedback from modules;
/// which fields are populated depends on the module type and various other settings.
///
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
///
/// The subobjects contain references to the parent feedback object, and so should
/// not be used after the parent object has been destroyed.
///
/// The fields in the feedback object are typed; generally, these are optional-style
/// read-only fields (i.e., have the concept of has/get), although the return types
/// and exact interface vary slightly between fields. Where appropriate, the explicit
/// bool operator has been overridden so that you can shortcut @c if(field.has()) by
/// calling @c if(field).
///
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Feedback final {
public:
  enum class TemperatureState {
    /// Temperature within normal range
    Normal,
    /// Motor output beginning to be limited due to high temperature
    Critical,
    /// Temperature exceeds max allowable for motor; motor output disabled
    ExceedMaxMotor,
    /// Temperature exceeds max allowable for electronics; motor output disabled
    ExceedMaxBoard,
  };

  enum class MstopState {
    /// The MStop is pressed
    Triggered,
    /// The MStop is not pressed
    NotTriggered,
  };

  enum class PositionLimitState {
    /// The position of the module was below the lower safety limit; the motor output is set to return the module to
    /// within the limits
    Below,
    /// The position of the module was near the lower safety limit, and the motor output is being limited or reversed
    AtLower,
    /// The position of the module was within the safety limits
    Inside,
    /// The position of the module was near the upper safety limit, and the motor output is being limited or reversed
    AtUpper,
    /// The position of the module was above the upper safety limit; the motor output is set to return the module to
    /// within the limits
    Above,
    /// The module has not been inside the safety limits since it was booted or the safety limits were set
    Uninitialized,
  };

  enum class VelocityLimitState {
    /// The velocity of the module was below the lower safety limit; the motor output is set to return the module to
    /// within the limits
    Below,
    /// The velocity of the module was near the lower safety limit, and the motor output is being limited or reversed
    AtLower,
    /// The velocity of the module was within the safety limits
    Inside,
    /// The velocity of the module was near the upper safety limit, and the motor output is being limited or reversed
    AtUpper,
    /// The velocity of the module was above the upper safety limit; the motor output is set to return the module to
    /// within the limits
    Above,
    /// The module has not been inside the safety limits since it was booted or the safety limits were set
    Uninitialized,
  };

  enum class EffortLimitState {
    /// The effort of the module was below the lower safety limit; the motor output is set to return the module to
    /// within the limits
    Below,
    /// The effort of the module was near the lower safety limit, and the motor output is being limited or reversed
    AtLower,
    /// The effort of the module was within the safety limits
    Inside,
    /// The effort of the module was near the upper safety limit, and the motor output is being limited or reversed
    AtUpper,
    /// The effort of the module was above the upper safety limit; the motor output is set to return the module to
    /// within the limits
    Above,
    /// The module has not been inside the safety limits since it was booted or the safety limits were set
    Uninitialized,
  };

  enum class CommandLifetimeState {
    /// There is not command lifetime active on this module
    Unlocked,
    /// Commands are locked out due to control from other users
    LockedByOther,
    /// Commands from others are locked out due to control from this group
    LockedBySender,
  };

  enum class ArQuality {
    /// Camera position tracking is not available.
    ArQualityNotAvailable,
    /// Tracking is available albeit suboptimal for an unknown reason.
    ArQualityLimitedUnknown,
    /// The AR session has not yet gathered enough camera or motion data to provide tracking information.
    ArQualityLimitedInitializing,
    /// The AR session is attempting to resume after an interruption.
    ArQualityLimitedRelocalizing,
    /// The device is moving too fast for accurate image-based position tracking.
    ArQualityLimitedExcessiveMotion,
    /// The scene visible to the camera does not contain enough distinguishable features for image-based position
    /// tracking.
    ArQualityLimitedInsufficientFeatures,
    /// Camera position tracking is providing optimal results.
    ArQualityNormal,
  };

protected:
  /// \brief A message field representable by a single-precision floating point value.
  class FloatField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    FloatField(const HebiFeedbackRef& internal, HebiFeedbackFloatField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::FloatField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    float get() const;

    HEBI_DISABLE_COPY_MOVE(FloatField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackFloatField const field_;
  };

  /// \brief A message field for an angle measurement which does not lose
  /// precision at very high angles.
  ///
  /// This field is represented as an int64_t for the number of revolutions
  /// and a float for the radian offset from that number of revolutions.
  class HighResAngleField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    HighResAngleField(const HebiFeedbackRef& internal, HebiFeedbackHighResAngleField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::HighResAngleField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value as a double;
    /// otherwise, returns a default.
    ///
    /// Note that some precision might be lost converting to a double at
    /// very high number of revolutions.
    double get() const;
    /// \brief If the field has a value, returns that value in the int64
    /// and float parameters passed in; otherwise, returns a default.
    ///
    /// Note that this maintains the full precision of the underlying angle
    /// measurement, even for very large numbers of revolutions.
    ///
    /// \param revolutions The number of full revolutions
    ///
    /// \param radian_offset The offset from the given number of full
    /// revolutions.  Note that this is usually between 0 and @c 2*M_PI, but
    /// callers should not assume this.
    void get(int64_t* revolutions, float* radian_offset) const;

    HEBI_DISABLE_COPY_MOVE(HighResAngleField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackHighResAngleField const field_;
  };

  /// \brief A message field containing a numbered set of single-precision
  /// floating point values.
  class NumberedFloatField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    NumberedFloatField(const HebiFeedbackRef& internal, HebiFeedbackNumberedFloatField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief True if (and only if) the particular numbered subvalue of
    /// this field has a value.
    ///
    /// \param fieldNumber Which subvalue to check; valid values for
    /// fieldNumber depend on the field type.
    bool has(size_t fieldNumber) const;
    /// \brief If the particular numbered subvalue of this field has a
    /// value, returns that value; otherwise returns a default.
    ///
    /// \param fieldNumber Which subvalue to get; valid values for
    /// fieldNumber depend on the field type.
    float get(size_t fieldNumber) const;

    HEBI_DISABLE_COPY_MOVE(NumberedFloatField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackNumberedFloatField const field_;
  };

  /// \brief A message field representable by an unsigned 64 bit integer value.
  class UInt64Field final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    UInt64Field(const HebiFeedbackRef& internal, HebiFeedbackUInt64Field field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::UInt64Field& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    uint64_t get() const;

    HEBI_DISABLE_COPY_MOVE(UInt64Field)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackUInt64Field const field_;
  };

  /// \brief A message field representable by a 3-D vector of single-precision
  /// floating point values.
  class Vector3fField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Vector3fField(const HebiFeedbackRef& internal, HebiFeedbackVector3fField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::Vector3fField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value!" << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    Vector3f get() const;

    HEBI_DISABLE_COPY_MOVE(Vector3fField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackVector3fField const field_;
  };

  /// \brief A message field representable by a 3-D vector of single-precision
  /// floating point values.
  class QuaternionfField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    QuaternionfField(const HebiFeedbackRef& internal, HebiFeedbackQuaternionfField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::QuaternionfField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value!" << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    Quaternionf get() const;

    HEBI_DISABLE_COPY_MOVE(QuaternionfField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackQuaternionfField const field_;
  };

  /// \brief A message field representable by an enum of a given type.
  template<typename T>
  class EnumField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    EnumField(const HebiFeedbackRef& internal, HebiFeedbackEnumField field) : internal_(internal), field_(field) {}
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::EnumField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const {
      return (enumGetter(internal_, field_, nullptr) == HebiStatusSuccess);
    }
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    T get() const {
      int32_t ret{};
      enumGetter(internal_, field_, &ret);
      return static_cast<T>(ret);
    }

    HEBI_DISABLE_COPY_MOVE(EnumField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackEnumField const field_;
  };

  /// \brief A message field for interfacing with a bank of I/O pins.
  class IoBank final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    IoBank(const HebiFeedbackRef& internal, HebiFeedbackIoPinBank bank);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief True if (and only if) the particular numbered pin in this
    /// bank has an integer (e.g., digital) value.
    ///
    /// \param pinNumber Which pin to check; valid values for pinNumber
    /// depend on the bank.
    bool hasInt(size_t pinNumber) const;
    /// \brief True if (and only if) the particular numbered pin in this
    /// bank has an floating point (e.g., analog or PWM) value.
    ///
    /// \param pinNumber Which pin to check; valid values for pinNumber
    /// depend on the bank.
    bool hasFloat(size_t pinNumber) const;
    /// \brief If this numbered pin in this bank has an integer (e.g.,
    /// digital) value, returns that value; otherwise returns a default.
    ///
    /// \param pinNumber Which pin to get; valid values for pinNumber
    /// depend on the bank.
    int64_t getInt(size_t pinNumber) const;
    /// \brief If this numbered pin in this bank has an floating point
    /// (e.g., analog or PWM) value, returns that value; otherwise returns a
    /// default.
    ///
    /// \param pinNumber Which pin to get; valid values for pinNumber
    /// depend on the bank.
    float getFloat(size_t pinNumber) const;

    HEBI_DISABLE_COPY_MOVE(IoBank)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackIoPinBank const bank_;
  };

  /// \brief A message field for interfacing with an LED.
  class LedField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    LedField(const HebiFeedbackRef& internal, HebiFeedbackLedField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the LED color is set
    /// without directly calling @c hasColor().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Feedback::LedField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has color!" << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return hasColor(); }
    /// \brief Returns true if the LED color is set, and false otherwise.
    bool hasColor() const;
    /// \brief Returns the led color.
    Color getColor() const;

    HEBI_DISABLE_COPY_MOVE(LedField)
  private:
    const HebiFeedbackRef& internal_;
    HebiFeedbackLedField const field_;
  };

  /// Feedback from any available I/O pins on the device.
  class Io final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Io(const HebiFeedbackRef& internal)
      : internal_(internal),
        a_(internal, HebiFeedbackIoBankA),
        b_(internal, HebiFeedbackIoBankB),
        c_(internal, HebiFeedbackIoBankC),
        d_(internal, HebiFeedbackIoBankD),
        e_(internal, HebiFeedbackIoBankE),
        f_(internal, HebiFeedbackIoBankF) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Subfields ----------------

    /// I/O pin bank a (pins 1-8 available)
    const IoBank& a() const { return a_; }
    /// I/O pin bank b (pins 1-8 available)
    const IoBank& b() const { return b_; }
    /// I/O pin bank c (pins 1-8 available)
    const IoBank& c() const { return c_; }
    /// I/O pin bank d (pins 1-8 available)
    const IoBank& d() const { return d_; }
    /// I/O pin bank e (pins 1-8 available)
    const IoBank& e() const { return e_; }
    /// I/O pin bank f (pins 1-8 available)
    const IoBank& f() const { return f_; }

    HEBI_DISABLE_COPY_MOVE(Io)
  private:
     const HebiFeedbackRef& internal_;

    IoBank a_;
    IoBank b_;
    IoBank c_;
    IoBank d_;
    IoBank e_;
    IoBank f_;
  };

  /// Actuator-specific feedback.
  class Actuator final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Actuator(const HebiFeedbackRef& internal)
      : velocity_(internal, HebiFeedbackFloatVelocity),
        effort_(internal, HebiFeedbackFloatEffort),
        velocity_command_(internal, HebiFeedbackFloatVelocityCommand),
        effort_command_(internal, HebiFeedbackFloatEffortCommand),
        deflection_(internal, HebiFeedbackFloatDeflection),
        deflection_velocity_(internal, HebiFeedbackFloatDeflectionVelocity),
        motor_velocity_(internal, HebiFeedbackFloatMotorVelocity),
        motor_current_(internal, HebiFeedbackFloatMotorCurrent),
        motor_sensor_temperature_(internal, HebiFeedbackFloatMotorSensorTemperature),
        motor_winding_current_(internal, HebiFeedbackFloatMotorWindingCurrent),
        motor_winding_temperature_(internal, HebiFeedbackFloatMotorWindingTemperature),
        motor_housing_temperature_(internal, HebiFeedbackFloatMotorHousingTemperature),
        pwm_command_(internal, HebiFeedbackFloatPwmCommand),
        position_(internal, HebiFeedbackHighResAnglePosition),
        position_command_(internal, HebiFeedbackHighResAnglePositionCommand),
        motor_position_(internal, HebiFeedbackHighResAngleMotorPosition),
        sequence_number_(internal, HebiFeedbackUInt64SequenceNumber),
        temperature_state_(internal, HebiFeedbackEnumTemperatureState),
        mstop_state_(internal, HebiFeedbackEnumMstopState),
        position_limit_state_(internal, HebiFeedbackEnumPositionLimitState),
        velocity_limit_state_(internal, HebiFeedbackEnumVelocityLimitState),
        effort_limit_state_(internal, HebiFeedbackEnumEffortLimitState),
        command_lifetime_state_(internal, HebiFeedbackEnumCommandLifetimeState) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Subfields ----------------

    /// Velocity of the module output (post-spring), in radians/second.
    const FloatField& velocity() const { return velocity_; }
    /// Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
    const FloatField& effort() const { return effort_; }
    /// Commanded velocity of the module output (post-spring), in radians/second.
    const FloatField& velocityCommand() const { return velocity_command_; }
    /// Commanded effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
    const FloatField& effortCommand() const { return effort_command_; }
    /// Difference (in radians) between the pre-spring and post-spring output position.
    const FloatField& deflection() const { return deflection_; }
    /// Velocity (in radians/second) of the difference between the pre-spring and post-spring output position.
    const FloatField& deflectionVelocity() const { return deflection_velocity_; }
    /// The velocity (in radians/second) of the motor shaft.
    const FloatField& motorVelocity() const { return motor_velocity_; }
    /// Current supplied to the motor.
    const FloatField& motorCurrent() const { return motor_current_; }
    /// The temperature from a sensor near the motor housing.
    const FloatField& motorSensorTemperature() const { return motor_sensor_temperature_; }
    /// The estimated current in the motor windings.
    const FloatField& motorWindingCurrent() const { return motor_winding_current_; }
    /// The estimated temperature of the motor windings.
    const FloatField& motorWindingTemperature() const { return motor_winding_temperature_; }
    /// The estimated temperature of the motor housing.
    const FloatField& motorHousingTemperature() const { return motor_housing_temperature_; }
    /// Commanded PWM signal sent to the motor; final output of PID controllers.
    const FloatField& pwmCommand() const { return pwm_command_; }
    /// Position of the module output (post-spring), in radians.
    const HighResAngleField& position() const { return position_; }
    /// Commanded position of the module output (post-spring), in radians.
    const HighResAngleField& positionCommand() const { return position_command_; }
    /// The position of an actuator’s internal motor before the gear reduction, in radians.
    const HighResAngleField& motorPosition() const { return motor_position_; }
    /// Sequence number going to module (local)
    const UInt64Field& sequenceNumber() const { return sequence_number_; }
    /// Describes how the temperature inside the module is limiting the output of the motor
    const EnumField<TemperatureState>& temperatureState() const { return temperature_state_; }
    /// Current status of the MStop
    const EnumField<MstopState>& mstopState() const { return mstop_state_; }
    /// Software-controlled bounds on the allowable position of the module; user settable
    const EnumField<PositionLimitState>& positionLimitState() const { return position_limit_state_; }
    /// Software-controlled bounds on the allowable velocity of the module
    const EnumField<VelocityLimitState>& velocityLimitState() const { return velocity_limit_state_; }
    /// Software-controlled bounds on the allowable effort of the module
    const EnumField<EffortLimitState>& effortLimitState() const { return effort_limit_state_; }
    /// The state of the command lifetime safety controller, with respect to the current group
    const EnumField<CommandLifetimeState>& commandLifetimeState() const { return command_lifetime_state_; }

    HEBI_DISABLE_COPY_MOVE(Actuator)
  private:
    FloatField velocity_;
    FloatField effort_;
    FloatField velocity_command_;
    FloatField effort_command_;
    FloatField deflection_;
    FloatField deflection_velocity_;
    FloatField motor_velocity_;
    FloatField motor_current_;
    FloatField motor_sensor_temperature_;
    FloatField motor_winding_current_;
    FloatField motor_winding_temperature_;
    FloatField motor_housing_temperature_;
    FloatField pwm_command_;
    HighResAngleField position_;
    HighResAngleField position_command_;
    HighResAngleField motor_position_;
    UInt64Field sequence_number_;
    EnumField<TemperatureState> temperature_state_;
    EnumField<MstopState> mstop_state_;
    EnumField<PositionLimitState> position_limit_state_;
    EnumField<VelocityLimitState> velocity_limit_state_;
    EnumField<EffortLimitState> effort_limit_state_;
    EnumField<CommandLifetimeState> command_lifetime_state_;
  };

  /// Feedback generally from a mobile device such as a phone or tablet.
  class Mobile final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Mobile(const HebiFeedbackRef& internal)
      : battery_level_(internal, HebiFeedbackFloatBatteryLevel),
        ar_position_(internal, HebiFeedbackVector3fArPosition),
        ar_orientation_(internal, HebiFeedbackQuaternionfArOrientation),
        ar_quality_(internal, HebiFeedbackEnumArQuality) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Subfields ----------------

    /// Charge level of the device’s battery (in percent).
    const FloatField& batteryLevel() const { return battery_level_; }
    /// A device's position in the world as calculated from an augmented reality framework, in meters
    const Vector3fField& arPosition() const { return ar_position_; }
    /// A device's orientation in the world as calculated from an augmented reality framework
    const QuaternionfField& arOrientation() const { return ar_orientation_; }
    /// The status of the augmented reality tracking, if using an AR enabled device.
    const EnumField<ArQuality>& arQuality() const { return ar_quality_; }

    HEBI_DISABLE_COPY_MOVE(Mobile)
  private:
    FloatField battery_level_;
    Vector3fField ar_position_;
    QuaternionfField ar_orientation_;
    EnumField<ArQuality> ar_quality_;
  };

  /// Inertial measurement unit feedback (accelerometers and gyros).
  class Imu final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Imu(const HebiFeedbackRef& internal)
      : accelerometer_(internal, HebiFeedbackVector3fAccelerometer),
        gyro_(internal, HebiFeedbackVector3fGyro),
        orientation_(internal, HebiFeedbackQuaternionfOrientation) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Subfields ----------------

    /// Accelerometer data, in m/s^2.
    const Vector3fField& accelerometer() const { return accelerometer_; }
    /// Gyro data, in radians/second.
    const Vector3fField& gyro() const { return gyro_; }
    /// A filtered estimate of the orientation of the module.
    const QuaternionfField& orientation() const { return orientation_; }

    HEBI_DISABLE_COPY_MOVE(Imu)
  private:
    Vector3fField accelerometer_;
    Vector3fField gyro_;
    QuaternionfField orientation_;
  };

private:
  /**
   * C-style object; managed by parent.
   * NOTE: this should not be used except by internal library functions!
   */
  HebiFeedbackPtr internal_;
  HebiFeedbackRef internal_ref_;

public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Wraps an existing C-style object that is managed by its parent.
   * NOTE: this should not be used except by internal library functions!
   */
  Feedback(HebiFeedbackPtr);
#endif // DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Move constructor (necessary for containment in STL template classes)
   */
  Feedback(Feedback&& other);

  // With all submessage and field getters: Note that the returned reference
  // should not be used after the lifetime of this parent.

  // Submessages -------------------------------------------------------------

  /// Feedback from any available I/O pins on the device.
  const Io& io() const { return io_; }
  /// Actuator-specific feedback.
  const Actuator& actuator() const { return actuator_; }
  /// Feedback generally from a mobile device such as a phone or tablet.
  const Mobile& mobile() const { return mobile_; }
  /// Inertial measurement unit feedback (accelerometers and gyros).
  const Imu& imu() const { return imu_; }

  // Subfields -------------------------------------------------------------

  /// Ambient temperature inside the module (measured at the IMU chip), in degrees Celsius.
  const FloatField& boardTemperature() const { return board_temperature_; }
  /// Temperature of the processor chip, in degrees Celsius.
  const FloatField& processorTemperature() const { return processor_temperature_; }
  /// Bus voltage that the module is running at (in Volts).
  const FloatField& voltage() const { return voltage_; }
#ifndef DOXYGEN_OMIT_INTERNAL
  /// Values for internal debug functions (channel 1-9 available).
  const NumberedFloatField& debug() const { return debug_; }
#endif // DOXYGEN_OMIT_INTERNAL
  /// Timestamp of when message was received from module (local; microseconds)
  const UInt64Field& receiveTimeUs() const { return receive_time_us_; }
  /// Timestamp of when message was transmitted to module (local; microseconds)
  const UInt64Field& transmitTimeUs() const { return transmit_time_us_; }
  /// Timestamp of when message was received by module (remote; microseconds)
  const UInt64Field& hardwareReceiveTimeUs() const { return hardware_receive_time_us_; }
  /// Timestamp of when message was transmitted from module (remote; microseconds)
  const UInt64Field& hardwareTransmitTimeUs() const { return hardware_transmit_time_us_; }
  /// Unique ID of the module transmitting this feedback
  const UInt64Field& senderId() const { return sender_id_; }
  /// The module's LED.
  const LedField& led() const { return led_; }

  /**
   * Disable copy constructor/assignment operators
   */
  HEBI_DISABLE_COPY(Feedback)

  /* Disable move assigment operator. */
  Feedback& operator=(Feedback&& other) = delete;

private:
  Io io_;
  Actuator actuator_;
  Mobile mobile_;
  Imu imu_;

  FloatField board_temperature_;
  FloatField processor_temperature_;
  FloatField voltage_;
  NumberedFloatField debug_;
  UInt64Field receive_time_us_;
  UInt64Field transmit_time_us_;
  UInt64Field hardware_receive_time_us_;
  UInt64Field hardware_transmit_time_us_;
  UInt64Field sender_id_;
  LedField led_;
};

} // namespace hebi
