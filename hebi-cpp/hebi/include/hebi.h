#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

////////////////////////////////////////////////////////////////////////////////
/// Enum Types
////////////////////////////////////////////////////////////////////////////////

/**
 * API call status codes
 */
typedef enum HebiStatusCode {
  HebiStatusSuccess = 0, ///Success; no failures occurred
  HebiStatusInvalidArgument = 1, ///An invalid argument was supplied to the routine (e.g. null pointer)
  HebiStatusBufferTooSmall = 2, ///A buffer supplied to the routine was too small (normally determined by a size parameter)
  HebiStatusValueNotSet = 3, ///Returned when an accessor function attempts to retrieve a field which is not set
  HebiStatusFailure = 4, ///Generic code for failure; this is generally used for an internal or unknown failure
  HebiStatusArgumentOutOfRange = 5 ///Failure caused by an argument supplied to the routine that is out of range (e.g. a negative integer when only a positive integer is valid)
} HebiStatusCode;

/**
 * AR Quality codes
 */
typedef enum HebiArQuality {
  HebiArQualityNotAvailable, ///Camera position tracking is not available
  HebiArQualityLimitedUnknown, ///Tracking is available albeit suboptimal for an unknown reason
  HebiArQualityLimitedInitializing, ///The AR session has not yet gathered enough camera or motion data to provide tracking information
  HebiArQualityLimitedRelocalizing, ///The AR session is attempting to resume after an interruption
  HebiArQualityLimitedExcessiveMotion, ///The device is moving too fast for accurate image-based position tracking
  HebiArQualityLimitedInsufficientFeatures, ///The scene visible to the camera does not contain enough distinguishable features for image-based position tracking
  HebiArQualityNormal ///Camera position tracking is providing optimal results
} HebiArQuality;

typedef enum HebiIoBankPinResidentType {
  HebiIoBankPinResidentTypeNone, // Io pin should be interpreted empty
  HebiIoBankPinResidentTypeInteger, // Io pin should be interpreted as an integer (`int64_t`)
  HebiIoBankPinResidentTypeFloat // Io pin should be interpreted as a float (`float`)
} HebiIoBankPinResidentType;

////////////////////////////////////////////////////////////////////////////////
/// Command Enums
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiCommandFloatField {
  HebiCommandFloatVelocity, /// Velocity of the module output (post-spring).
  HebiCommandFloatEffort, /// Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
  HebiCommandFloatPositionKp, /// Proportional PID gain for position
  HebiCommandFloatPositionKi, /// Integral PID gain for position
  HebiCommandFloatPositionKd, /// Derivative PID gain for position
  HebiCommandFloatPositionFeedForward, /// Feed forward term for position (this term is multiplied by the target and added to the output).
  HebiCommandFloatPositionDeadZone, /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiCommandFloatPositionIClamp, /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiCommandFloatPositionPunch, /// Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiCommandFloatPositionMinTarget,     /// Minimum allowed value for input to the PID controller
  HebiCommandFloatPositionMaxTarget,     /// Maximum allowed value for input to the PID controller
  HebiCommandFloatPositionTargetLowpass, /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatPositionMinOutput,     /// Output from the PID controller is limited to a minimum of this value.
  HebiCommandFloatPositionMaxOutput,     /// Output from the PID controller is limited to a maximum of this value.
  HebiCommandFloatPositionOutputLowpass, /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatVelocityKp,            /// Proportional PID gain for velocity
  HebiCommandFloatVelocityKi,            /// Integral PID gain for velocity
  HebiCommandFloatVelocityKd,            /// Derivative PID gain for velocity
  HebiCommandFloatVelocityFeedForward, /// Feed forward term for velocity (this term is multiplied by the target and added to the output).
  HebiCommandFloatVelocityDeadZone, /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiCommandFloatVelocityIClamp, /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiCommandFloatVelocityPunch, /// Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiCommandFloatVelocityMinTarget,     /// Minimum allowed value for input to the PID controller
  HebiCommandFloatVelocityMaxTarget,     /// Maximum allowed value for input to the PID controller
  HebiCommandFloatVelocityTargetLowpass, /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatVelocityMinOutput,     /// Output from the PID controller is limited to a minimum of this value.
  HebiCommandFloatVelocityMaxOutput,     /// Output from the PID controller is limited to a maximum of this value.
  HebiCommandFloatVelocityOutputLowpass, /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatEffortKp,              /// Proportional PID gain for effort
  HebiCommandFloatEffortKi,              /// Integral PID gain for effort
  HebiCommandFloatEffortKd,              /// Derivative PID gain for effort
  HebiCommandFloatEffortFeedForward, /// Feed forward term for effort (this term is multiplied by the target and added to the output).
  HebiCommandFloatEffortDeadZone, /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiCommandFloatEffortIClamp, /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiCommandFloatEffortPunch, /// Constant offset to the effort PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiCommandFloatEffortMinTarget,     /// Minimum allowed value for input to the PID controller
  HebiCommandFloatEffortMaxTarget,     /// Maximum allowed value for input to the PID controller
  HebiCommandFloatEffortTargetLowpass, /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatEffortMinOutput,     /// Output from the PID controller is limited to a minimum of this value.
  HebiCommandFloatEffortMaxOutput,     /// Output from the PID controller is limited to a maximum of this value.
  HebiCommandFloatEffortOutputLowpass, /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatSpringConstant,      /// The spring constant of the module.
  HebiCommandFloatReferencePosition, /// Set the internal encoder reference offset so that the current position matches the given reference command
  HebiCommandFloatReferenceEffort, /// Set the internal effort reference offset so that the current effort matches the given reference command
  HebiCommandFloatVelocityLimitMin, /// The firmware safety limit for the minimum allowed velocity.
  HebiCommandFloatVelocityLimitMax, /// The firmware safety limit for the maximum allowed velocity.
  HebiCommandFloatEffortLimitMin,   /// The firmware safety limit for the minimum allowed effort.
  HebiCommandFloatEffortLimitMax,   /// The firmware safety limit for the maximum allowed effort.
} HebiCommandFloatField;

typedef enum HebiCommandHighResAngleField {
  HebiCommandHighResAnglePosition,         /// Position of the module output (post-spring).
  HebiCommandHighResAnglePositionLimitMin, /// The firmware safety limit for the minimum allowed position.
  HebiCommandHighResAnglePositionLimitMax, /// The firmware safety limit for the maximum allowed position.
} HebiCommandHighResAngleField;

typedef enum HebiCommandNumberedFloatField {
  HebiCommandNumberedFloatDebug, /// Values for internal debug functions (channel 1-9 available).
} HebiCommandNumberedFloatField;

typedef enum HebiCommandBoolField {
  HebiCommandBoolPositionDOnError, /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement." When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiCommandBoolVelocityDOnError, /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement." When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiCommandBoolEffortDOnError, /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement." When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiCommandBoolAccelIncludesGravity, /// Whether to include acceleration due to gravity in acceleration feedback.
} HebiCommandBoolField;

typedef enum HebiCommandStringField {
  HebiCommandStringName,   /// The name for this module. The string must be null-terminated and less than 21 characters.
  HebiCommandStringFamily, /// The family for this module. The string must be null-terminated and less than 21 characters.
  HebiCommandStringAppendLog, /// Appends to the current log message on the module.
} HebiCommandStringField;

typedef enum HebiCommandFlagField {
  HebiCommandFlagSaveCurrentSettings, /// Indicates if the module should save the current values of all of its settings.
  HebiCommandFlagReset,               /// Restart the module.
  HebiCommandFlagBoot,                /// Boot the module from bootloader into application.
  HebiCommandFlagStopBoot,            /// Stop the module from automatically booting into application.
  HebiCommandFlagClearLog,            /// Clears the log message on the module.
} HebiCommandFlagField;

typedef enum HebiCommandEnumField {
  HebiCommandEnumControlStrategy, /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
  HebiCommandEnumMstopStrategy,            /// The motion stop strategy for the actuator
  HebiCommandEnumMinPositionLimitStrategy, /// The position limit strategy (at the minimum position) for the actuator
  HebiCommandEnumMaxPositionLimitStrategy, /// The position limit strategy (at the maximum position) for the actuator
} HebiCommandEnumField;

typedef enum HebiCommandIoPinBank {
  HebiCommandIoBankA, /// I/O pin bank a (pins 1-8 available)
  HebiCommandIoBankB, /// I/O pin bank b (pins 1-8 available)
  HebiCommandIoBankC, /// I/O pin bank c (pins 1-8 available)
  HebiCommandIoBankD, /// I/O pin bank d (pins 1-8 available)
  HebiCommandIoBankE, /// I/O pin bank e (pins 1-8 available)
  HebiCommandIoBankF, /// I/O pin bank f (pins 1-8 available)
} HebiCommandIoPinBank;

typedef enum HebiCommandLedField {
  HebiCommandLedLed, /// The module's LED.
} HebiCommandLedField;

////////////////////////////////////////////////////////////////////////////////
/// Feedback Enums
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiFeedbackFloatField {
  HebiFeedbackFloatBoardTemperature,     /// Ambient temperature inside the module (measured at the IMU chip)
  HebiFeedbackFloatProcessorTemperature, /// Temperature of the processor chip.
  HebiFeedbackFloatVoltage,              /// Bus voltage at which the module is running.
  HebiFeedbackFloatVelocity,             /// Velocity of the module output (post-spring).
  HebiFeedbackFloatEffort, /// Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
  HebiFeedbackFloatVelocityCommand, /// Commanded velocity of the module output (post-spring)
  HebiFeedbackFloatEffortCommand, /// Commanded effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
  HebiFeedbackFloatDeflection,         /// Difference between the pre-spring and post-spring output position.
  HebiFeedbackFloatDeflectionVelocity, /// Velocity of the difference between the pre-spring and post-spring output position.
  HebiFeedbackFloatMotorVelocity,      /// The velocity of the motor shaft.
  HebiFeedbackFloatMotorCurrent,       /// Current supplied to the motor.
  HebiFeedbackFloatMotorSensorTemperature,  /// The temperature from a sensor near the motor housing.
  HebiFeedbackFloatMotorWindingCurrent,     /// The estimated current in the motor windings.
  HebiFeedbackFloatMotorWindingTemperature, /// The estimated temperature of the motor windings.
  HebiFeedbackFloatMotorHousingTemperature, /// The estimated temperature of the motor housing.
  HebiFeedbackFloatBatteryLevel,            /// Charge level of the device’s battery (in percent).
  HebiFeedbackFloatPwmCommand,              /// Commanded PWM signal sent to the motor; final output of PID controllers.
} HebiFeedbackFloatField;

typedef enum HebiFeedbackHighResAngleField {
  HebiFeedbackHighResAnglePosition,        /// Position of the module output (post-spring).
  HebiFeedbackHighResAnglePositionCommand, /// Commanded position of the module output (post-spring).
  HebiFeedbackHighResAngleMotorPosition,   /// The position of an actuator’s internal motor before the gear reduction.
} HebiFeedbackHighResAngleField;

typedef enum HebiFeedbackNumberedFloatField {
  HebiFeedbackNumberedFloatDebug, /// Values for internal debug functions (channel 1-9 available).
} HebiFeedbackNumberedFloatField;

typedef enum HebiFeedbackUInt64Field {
  HebiFeedbackUInt64SequenceNumber,       /// Sequence number going to module (local)
  HebiFeedbackUInt64ReceiveTime,          /// Timestamp of when message was received from module (local)
  HebiFeedbackUInt64TransmitTime,         /// Timestamp of when message was transmitted to module (local)
  HebiFeedbackUInt64HardwareReceiveTime,  /// Timestamp of when message was received by module (remote)
  HebiFeedbackUInt64HardwareTransmitTime, /// Timestamp of when message was transmitted from module (remote)
  HebiFeedbackUInt64SenderId,             /// Unique ID of the module transmitting this feedback
} HebiFeedbackUInt64Field;

typedef enum HebiFeedbackVector3fField {
  HebiFeedbackVector3fAccelerometer, /// Accelerometer data
  HebiFeedbackVector3fGyro,          /// Gyro data
  HebiFeedbackVector3fArPosition, /// A device's position in the world as calculated from an augmented reality framework
} HebiFeedbackVector3fField;

typedef enum HebiFeedbackQuaternionfField {
  HebiFeedbackQuaternionfOrientation, /// A filtered estimate of the orientation of the module.
  HebiFeedbackQuaternionfArOrientation, /// A device's orientation in the world as calculated from an augmented reality framework
} HebiFeedbackQuaternionfField;

typedef enum HebiFeedbackEnumField {
  HebiFeedbackEnumTemperatureState, /// Describes how the temperature inside the module is limiting the output of the motor
  HebiFeedbackEnumMstopState,       /// Current status of the MStop
  HebiFeedbackEnumPositionLimitState, /// Software-controlled bounds on the allowable position of the module; user settable
  HebiFeedbackEnumVelocityLimitState, /// Software-controlled bounds on the allowable velocity of the module
  HebiFeedbackEnumEffortLimitState,   /// Software-controlled bounds on the allowable effort of the module
  HebiFeedbackEnumCommandLifetimeState, /// The state of the command lifetime safety controller, with respect to the current group
  HebiFeedbackEnumArQuality, /// The status of the augmented reality tracking, if using an AR enabled device. See HebiArQuality for values.
} HebiFeedbackEnumField;

typedef enum HebiFeedbackIoPinBank {
  HebiFeedbackIoBankA, /// I/O pin bank a (pins 1-8 available)
  HebiFeedbackIoBankB, /// I/O pin bank b (pins 1-8 available)
  HebiFeedbackIoBankC, /// I/O pin bank c (pins 1-8 available)
  HebiFeedbackIoBankD, /// I/O pin bank d (pins 1-8 available)
  HebiFeedbackIoBankE, /// I/O pin bank e (pins 1-8 available)
  HebiFeedbackIoBankF, /// I/O pin bank f (pins 1-8 available)
} HebiFeedbackIoPinBank;

typedef enum HebiFeedbackLedField {
  HebiFeedbackLedLed, /// The module's LED.
} HebiFeedbackLedField;

////////////////////////////////////////////////////////////////////////////////
/// Info Enums
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiInfoFloatField {
  HebiInfoFloatPositionKp, /// Proportional PID gain for position
  HebiInfoFloatPositionKi, /// Integral PID gain for position
  HebiInfoFloatPositionKd, /// Derivative PID gain for position
  HebiInfoFloatPositionFeedForward, /// Feed forward term for position (this term is multiplied by the target and added to the output).
  HebiInfoFloatPositionDeadZone, /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiInfoFloatPositionIClamp, /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiInfoFloatPositionPunch, /// Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiInfoFloatPositionMinTarget,     /// Minimum allowed value for input to the PID controller
  HebiInfoFloatPositionMaxTarget,     /// Maximum allowed value for input to the PID controller
  HebiInfoFloatPositionTargetLowpass, /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatPositionMinOutput,     /// Output from the PID controller is limited to a minimum of this value.
  HebiInfoFloatPositionMaxOutput,     /// Output from the PID controller is limited to a maximum of this value.
  HebiInfoFloatPositionOutputLowpass, /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatVelocityKp,            /// Proportional PID gain for velocity
  HebiInfoFloatVelocityKi,            /// Integral PID gain for velocity
  HebiInfoFloatVelocityKd,            /// Derivative PID gain for velocity
  HebiInfoFloatVelocityFeedForward, /// Feed forward term for velocity (this term is multiplied by the target and added to the output).
  HebiInfoFloatVelocityDeadZone, /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiInfoFloatVelocityIClamp, /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiInfoFloatVelocityPunch, /// Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiInfoFloatVelocityMinTarget,     /// Minimum allowed value for input to the PID controller
  HebiInfoFloatVelocityMaxTarget,     /// Maximum allowed value for input to the PID controller
  HebiInfoFloatVelocityTargetLowpass, /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatVelocityMinOutput,     /// Output from the PID controller is limited to a minimum of this value.
  HebiInfoFloatVelocityMaxOutput,     /// Output from the PID controller is limited to a maximum of this value.
  HebiInfoFloatVelocityOutputLowpass, /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatEffortKp,              /// Proportional PID gain for effort
  HebiInfoFloatEffortKi,              /// Integral PID gain for effort
  HebiInfoFloatEffortKd,              /// Derivative PID gain for effort
  HebiInfoFloatEffortFeedForward, /// Feed forward term for effort (this term is multiplied by the target and added to the output).
  HebiInfoFloatEffortDeadZone, /// Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiInfoFloatEffortIClamp, /// Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiInfoFloatEffortPunch, /// Constant offset to the effort PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiInfoFloatEffortMinTarget,     /// Minimum allowed value for input to the PID controller
  HebiInfoFloatEffortMaxTarget,     /// Maximum allowed value for input to the PID controller
  HebiInfoFloatEffortTargetLowpass, /// A simple lowpass filter applied to the target set point; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatEffortMinOutput,     /// Output from the PID controller is limited to a minimum of this value.
  HebiInfoFloatEffortMaxOutput,     /// Output from the PID controller is limited to a maximum of this value.
  HebiInfoFloatEffortOutputLowpass, /// A simple lowpass filter applied to the controller output; needs to be between 0 and 1. At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatSpringConstant,      /// The spring constant of the module.
  HebiInfoFloatVelocityLimitMin,    /// The firmware safety limit for the minimum allowed velocity.
  HebiInfoFloatVelocityLimitMax,    /// The firmware safety limit for the maximum allowed velocity.
  HebiInfoFloatEffortLimitMin,      /// The firmware safety limit for the minimum allowed effort.
  HebiInfoFloatEffortLimitMax,      /// The firmware safety limit for the maximum allowed effort.
} HebiInfoFloatField;

typedef enum HebiInfoHighResAngleField {
  HebiInfoHighResAnglePositionLimitMin, /// The firmware safety limit for the minimum allowed position.
  HebiInfoHighResAnglePositionLimitMax, /// The firmware safety limit for the maximum allowed position.
} HebiInfoHighResAngleField;

typedef enum HebiInfoBoolField {
  HebiInfoBoolPositionDOnError, /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement." When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiInfoBoolVelocityDOnError, /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement." When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiInfoBoolEffortDOnError, /// Controls whether the Kd term uses the "derivative of error" or "derivative of measurement." When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiInfoBoolAccelIncludesGravity, /// Whether to include acceleration due to gravity in acceleration feedback.
} HebiInfoBoolField;

typedef enum HebiInfoStringField {
  HebiInfoStringName,   /// The name for this module. The string must be null-terminated and less than 21 characters.
  HebiInfoStringFamily, /// The family for this module. The string must be null-terminated and less than 21 characters.
  HebiInfoStringSerial, /// Gets the serial number for this module (e.g., X5-0001).
} HebiInfoStringField;

typedef enum HebiInfoFlagField {
  HebiInfoFlagSaveCurrentSettings, /// Indicates if the module should save the current values of all of its settings.
} HebiInfoFlagField;

typedef enum HebiInfoEnumField {
  HebiInfoEnumControlStrategy, /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
  HebiInfoEnumCalibrationState,         /// The calibration state of the module
  HebiInfoEnumMstopStrategy,            /// The motion stop strategy for the actuator
  HebiInfoEnumMinPositionLimitStrategy, /// The position limit strategy (at the minimum position) for the actuator
  HebiInfoEnumMaxPositionLimitStrategy, /// The position limit strategy (at the maximum position) for the actuator
} HebiInfoEnumField;

typedef enum HebiInfoLedField {
  HebiInfoLedLed, /// The module's LED.
} HebiInfoLedField;

////////////////////////////////////////////////////////////////////////////////
/// RobotModel Enums
////////////////////////////////////////////////////////////////////////////////

/**
 * Which frame to report results in (e.g., for getForwardKinematics and other
 * functions.
 */
typedef enum HebiFrameType {
  HebiFrameTypeCenterOfMass,
  HebiFrameTypeOutput,
  HebiFrameTypeEndEffector,
  HebiFrameTypeInput
} HebiFrameType;

/**
 * Which type of robot model element (e.g. actuator, link, etc.)
 */

typedef enum HebiRobotModelElementType {
  HebiRobotModelElementTypeOther,
  HebiRobotModelElementTypeActuator,
  HebiRobotModelElementTypeBracket,
  HebiRobotModelElementTypeJoint,
  HebiRobotModelElementTypeLink,
  HebiRobotModelElementTypeRigidBody,
  HebiRobotModelElementTypeEndEffector
} HebiRobotModelElementType;

/**
 * What the type of motion (axis, rotation, translation, etc) is allowed by a
 * joint.
 */
typedef enum HebiJointType {
  HebiJointTypeRotationX,
  HebiJointTypeRotationY,
  HebiJointTypeRotationZ,
  HebiJointTypeTranslationX,
  HebiJointTypeTranslationY,
  HebiJointTypeTranslationZ
} HebiJointType;

/**
 * The type of actuators supported by the API
 */
typedef enum HebiActuatorType {
  HebiActuatorTypeX5_1,
  HebiActuatorTypeX5_4,
  HebiActuatorTypeX5_9,
  HebiActuatorTypeX8_3,
  HebiActuatorTypeX8_9,
  HebiActuatorTypeX8_16,
  HebiActuatorTypeR8_3,
  HebiActuatorTypeR8_9,
  HebiActuatorTypeR8_16
} HebiActuatorType;

/**
 * The type of links supported by the API
 */
typedef enum HebiLinkType {
  HebiLinkTypeX5,
  HebiLinkTypeR8,
} HebiLinkType;

/**
 * The type of link inputs supported by the API
 */
typedef enum HebiLinkInputType {
  HebiLinkInputTypeRightAngle,
  HebiLinkInputTypeInline
} HebiLinkInputType;

/**
 * The type of link outputs supported by the API
 */
typedef enum HebiLinkOutputType {
  HebiLinkOutputTypeRightAngle,
  HebiLinkOutputTypeInline
} HebiLinkOutputType;

/**
 * The type of brackets supported by the API
 */
typedef enum HebiBracketType {
  HebiBracketTypeX5LightLeft,
  HebiBracketTypeX5LightRight,
  HebiBracketTypeX5HeavyLeftInside,
  HebiBracketTypeX5HeavyLeftOutside,
  HebiBracketTypeX5HeavyRightInside,
  HebiBracketTypeX5HeavyRightOutside,
  HebiBracketTypeR8LightLeft,
  HebiBracketTypeR8LightRight,
  HebiBracketTypeR8HeavyLeftInside,
  HebiBracketTypeR8HeavyLeftOutside,
  HebiBracketTypeR8HeavyRightInside,
  HebiBracketTypeR8HeavyRightOutside
} HebiBracketType;

/**
 * The type of end effectors supported by the API
 */
typedef enum HebiEndEffectorType {
  HebiEndEffectorTypeCustom,
  HebiEndEffectorTypeX5Parallel,
  HebiEndEffectorTypeR8Parallel
} HebiEndEffectorType;

/**
 * How a matrix is serialized to an array.
 */
typedef enum HebiMatrixOrdering {
  HebiMatrixOrderingRowMajor,
  HebiMatrixOrderingColumnMajor
} HebiMatrixOrdering;

////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief The C-style's API representation of a command.
 *
 * Encapsulates data to be sent to a module
 */
typedef struct HebiCommand_* HebiCommandPtr;

/**
 * \brief The C-style's API representation of feedback.
 *
 * Encapsulates feedback received from a module
 */
typedef struct HebiFeedback_* HebiFeedbackPtr;

/**
 * \brief The C-style's API representation of info.
 *
 * Encapsulates info (settings, etc.) received from a module
 */
typedef struct HebiInfo_* HebiInfoPtr;

/**
 * \brief The C-style's API representation of a command object for a
 * group of modules.
 *
 * This is an iterable list of command structures; commands are sent to modules in a group using the
 * fields within this structure.
 */
typedef struct HebiGroupCommand_* HebiGroupCommandPtr;

/**
 * \brief The C-style's API representation of a feedback object for a
 * group of modules.
 *
 * This is an iterable list of feedback structures; feedback from modules in a group are retrieved primarily through
 * this structure.
 */
typedef struct HebiGroupFeedback_* HebiGroupFeedbackPtr;

/**
 * \brief The C-style's API representation of a info object for a
 * group of modules.
 *
 * This is an iterable list of info structures; info from modules in a group are retrieved primarily through
 * this structure.
 */
typedef struct HebiGroupInfo_* HebiGroupInfoPtr;

/**
 * \brief The C-style's API representation of a group.
 *
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 */
typedef struct HebiGroup_* HebiGroupPtr;

/**
 * Maintains a registry of network-connected modules and returns Group objects
 * to the user. Only one Lookup object is needed per application.
 *
 */
typedef struct HebiLookup_* HebiLookupPtr;

/**
 * A list of entries that represent a snapshot of the state of the lookup object
 * at some point in time.  These entries include network HEBI devices such
 * as actuators.
 */
typedef struct HebiLookupEntryList_* HebiLookupEntryListPtr;

/**
 * \brief The C-style's API representation of a log file.
 *
 * Represents a log file generated by the API.
 */
typedef struct HebiLogFile_* HebiLogFilePtr;

/**
 * \brief The C-style's API representation of a string.
 *
 * Represents a null terminated UTF-8 string
 */
typedef struct HebiString_* HebiStringPtr;

/**
 * A robot model object which stores a tree of connected modules, and allows for
 * computation of forward kinematics, jacobians, and more.
 */
typedef struct HebiRobotModel_* HebiRobotModelPtr;

/**
 * Contains a robot model element, which has an input and zero or more outputs.
 * This may refer to a rigid body or a massless joint.
 */
typedef struct HebiRobotModelElement_* HebiRobotModelElementPtr;

/**
 * An inverse kinematics object which uses a kinematics object to search for
 * joint angles that optimize any of several objectives while respecting defined
 * constraints.
 */
typedef struct HebiIK_* HebiIKPtr;

/**
 * \brief The C-style's API representation of a trajectory.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * Used to generate position, velocity, and acceleration for different joints.
 */
typedef struct HebiTrajectory_* HebiTrajectoryPtr;

/**
 * \brief Group feedback handling function signature
 */
typedef void (*GroupFeedbackHandlerFunction)(HebiGroupFeedbackPtr fbk, void* user_data);

////////////////////////////////////////////////////////////////////////////////
/// Structures
////////////////////////////////////////////////////////////////////////////////

typedef struct HebiMacAddress_ {
  uint8_t bytes_[6];
} HebiMacAddress;

typedef struct HebiVector3f_ {
  float x;
  float y;
  float z;
} HebiVector3f;

typedef struct HebiQuaternionf_ {
  float w;
  float x;
  float y;
  float z;
} HebiQuaternionf;

/**
 * Contains matadata information about a robot element (e.g. element is an X5-9 actuator)
 */
typedef struct HebiRobotModelElementMetadata_ {
  uint32_t struct_size_;
  HebiRobotModelElementType element_type_;
  union {
    struct /*Actuator Type */ {
      HebiActuatorType actuator_type_;
    };
    struct /*Bracket Type */ {
      HebiBracketType bracket_type_;
    };
    struct /*Joint Type */ {
      HebiJointType joint_type_;
    };
    struct /*Link Type*/ {
      HebiLinkType link_type_;
      HebiLinkInputType input_type_;
      HebiLinkOutputType output_type_;
      float extension_;
      float twist_;
    };
    struct /*End effector Type*/ {
      HebiEndEffectorType end_effector_type_;
    };
  };
} HebiRobotModelElementMetadata;

typedef struct HebiRobotModelElementTopology_ {
  int32_t element_index_;
  int32_t parent_index_;
  int32_t parent_output_;
  int32_t dof_location_;
  int32_t com_index_;
  int32_t output_index_;
  int32_t end_effector_index_;
} HebiRobotModelElementTopology;

typedef struct HebiHighResAngleStruct_ {
  int64_t revolutions_;
  float offset_;
} HebiHighResAngleStruct;

typedef struct HebiIoBankPinStruct_ {
  union {
    int64_t int_value_;
    float float_value_;    
  };
  HebiIoBankPinResidentType stored_type_;
} HebiIoBankPinStruct;

/**
 * Direct Command field reference
 */
typedef struct HebiCommandRef_ {
  int32_t* message_bitfield_;
  float* float_fields_;
  HebiHighResAngleStruct* high_res_angle_fields_;
  HebiVector3f* vector3f_fields_;
  HebiQuaternionf* quaternionf_fields_;
  uint64_t* uint64_fields_;
  int32_t* enum_fields_;
  bool* bool_fields_;
  float* numbered_float_fields_;
  HebiIoBankPinStruct* io_fields_;
  uint32_t* led_fields_;
  void* reserved_;
} HebiCommandRef;

/**
 * Command metadata which is guaranteed to be static for the loaded library
 */
typedef struct HebiCommandMetadata_ {
  // Total number of fields for each field type
  uint32_t float_field_count_;
  uint32_t high_res_angle_field_count_;
  uint32_t vector3f_field_count_;
  uint32_t quaternionf_field_count_;
  uint32_t uint64_field_count_;
  uint32_t enum_field_count_;
  uint32_t bool_field_count_;
  uint32_t numbered_float_field_count_;
  uint32_t io_field_count_;
  uint32_t led_field_count_;
  uint32_t string_field_count_;
  uint32_t flag_field_count_;
  // Field offsets into `message_bitfield_`
  uint32_t float_field_bitfield_offset_;
  uint32_t high_res_angle_field_bitfield_offset_;
  uint32_t vector3f_field_bitfield_offset_;
  uint32_t quaternionf_field_bitfield_offset_;
  uint32_t uint64_field_bitfield_offset_;
  uint32_t enum_field_bitfield_offset_;
  uint32_t bool_field_bitfield_offset_;
  uint32_t numbered_float_field_bitfield_offset_;
  uint32_t io_field_bitfield_offset_;
  uint32_t led_field_bitfield_offset_;
  uint32_t string_field_bitfield_offset_;
  uint32_t flag_field_bitfield_offset_;
  // Corresponds to the relative offset within the relevant block of `HebiCommandRef::message_bitfield_` for individual numbered float fields.
  const uint32_t* numbered_float_relative_offsets_;
  const uint32_t* numbered_float_field_sizes_;
  // Corresponds to the relative offset within the relevant block of `HebiCommandRef::message_bitfield_` for individual io bank pins.
  const uint32_t* io_relative_offsets_;
  const uint32_t* io_field_sizes_;
  // Total number of bits in `message_bitfield_`
  uint32_t message_bitfield_count_;
} HebiCommandMetadata;

/**
 * Direct Feedback field reference
 */
typedef struct HebiFeedbackRef_ {
  const int32_t* message_bitfield_;
  const float* float_fields_;
  const HebiHighResAngleStruct* high_res_angle_fields_;
  const HebiVector3f* vector3f_fields_;
  const HebiQuaternionf* quaternionf_fields_;
  const uint64_t* uint64_fields_;
  const int32_t* enum_fields_;
  const bool* bool_fields_;
  const float* numbered_float_fields_;
  const HebiIoBankPinStruct* io_fields_;
  const uint32_t* led_fields_;
  void* reserved_;
} HebiFeedbackRef;

/**
 * Feedback metadata which is guaranteed to be static for the loaded library
 */
typedef struct HebiFeedbackMetadata_ {
  // Total number of fields for each field type
  uint32_t float_field_count_;
  uint32_t high_res_angle_field_count_;
  uint32_t vector3f_field_count_;
  uint32_t quaternionf_field_count_;
  uint32_t uint64_field_count_;
  uint32_t enum_field_count_;
  uint32_t bool_field_count_;
  uint32_t numbered_float_field_count_;
  uint32_t io_field_count_;
  uint32_t led_field_count_;
  uint32_t string_field_count_;
  uint32_t flag_field_count_;
  // Field offsets into `message_bitfield_`
  uint32_t float_field_bitfield_offset_;
  uint32_t high_res_angle_field_bitfield_offset_;
  uint32_t vector3f_field_bitfield_offset_;
  uint32_t quaternionf_field_bitfield_offset_;
  uint32_t uint64_field_bitfield_offset_;
  uint32_t enum_field_bitfield_offset_;
  uint32_t bool_field_bitfield_offset_;
  uint32_t numbered_float_field_bitfield_offset_;
  uint32_t io_field_bitfield_offset_;
  uint32_t led_field_bitfield_offset_;
  uint32_t string_field_bitfield_offset_;
  uint32_t flag_field_bitfield_offset_;
  // Corresponds to the relative offset within the relevant block of `HebiFeedbackRef::message_bitfield_` for individual numbered float fields.
  const uint32_t* numbered_float_relative_offsets_;
  const uint32_t* numbered_float_field_sizes_;
  // Corresponds to the relative offset within the relevant block of `HebiFeedbackRef::message_bitfield_` for individual io bank pins.
  const uint32_t* io_relative_offsets_;
  const uint32_t* io_field_sizes_;
  // Total number of bits in `message_bitfield_`
  uint32_t message_bitfield_count_;
} HebiFeedbackMetadata;

/**
 * Direct Info field reference
 */
typedef struct HebiInfoRef_ {
  const int32_t* message_bitfield_;
  const float* float_fields_;
  const HebiHighResAngleStruct* high_res_angle_fields_;
  const HebiVector3f* vector3f_fields_;
  const HebiQuaternionf* quaternionf_fields_;
  const uint64_t* uint64_fields_;
  const int32_t* enum_fields_;
  const bool* bool_fields_;
  const float* numbered_float_fields_;
  const HebiIoBankPinStruct* io_fields_;
  const uint32_t* led_fields_;
  void* reserved_;
} HebiInfoRef;

/**
 * Info metadata which is guaranteed to be static for the loaded library
 */
typedef struct HebiInfoMetadata_ {
  // Total number of fields for each field type
  uint32_t float_field_count_;
  uint32_t high_res_angle_field_count_;
  uint32_t vector3f_field_count_;
  uint32_t quaternionf_field_count_;
  uint32_t uint64_field_count_;
  uint32_t enum_field_count_;
  uint32_t bool_field_count_;
  uint32_t numbered_float_field_count_;
  uint32_t io_field_count_;
  uint32_t led_field_count_;
  uint32_t string_field_count_;
  uint32_t flag_field_count_;
  // Field offsets into `message_bitfield_`
  uint32_t float_field_bitfield_offset_;
  uint32_t high_res_angle_field_bitfield_offset_;
  uint32_t vector3f_field_bitfield_offset_;
  uint32_t quaternionf_field_bitfield_offset_;
  uint32_t uint64_field_bitfield_offset_;
  uint32_t enum_field_bitfield_offset_;
  uint32_t bool_field_bitfield_offset_;
  uint32_t numbered_float_field_bitfield_offset_;
  uint32_t io_field_bitfield_offset_;
  uint32_t led_field_bitfield_offset_;
  uint32_t string_field_bitfield_offset_;
  uint32_t flag_field_bitfield_offset_;
  // Corresponds to the relative offset within the relevant block of `HebiInfoRef::message_bitfield_` for individual numbered float fields.
  const uint32_t* numbered_float_relative_offsets_;
  const uint32_t* numbered_float_field_sizes_;
  // Corresponds to the relative offset within the relevant block of `HebiInfoRef::message_bitfield_` for individual io bank pins.
  const uint32_t* io_relative_offsets_;
  const uint32_t* io_field_sizes_;
  // Total number of bits in `message_bitfield_`
  uint32_t message_bitfield_count_;
} HebiInfoMetadata;

////////////////////////////////////////////////////////////////////////////////
/// Lookup API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Create a Lookup instance.
 *
 * Lookup created by this function must be released with 'hebiLookupRelease'
 * when no longer needed.
 *
 * Note that this call invokes a background thread to query the network for
 * modules at regular intervals.
 * 
 * \param ifaces A buffer of c-style strings representing the interfaces
 *               to which to restrict the module discovery. The contents
 *               of the strings must be in human readable
 *               order (_e.g._, "192.168.1.0").
 *               If this parameter is null, then all interfaces besides
 *               the loopback interface are used to discover modules.
 *                                       
 * \param ifaces_length the length of the buffer `ifaces`. This must be zero
 *                      if `ifaces` is null.
 */
HebiLookupPtr hebiLookupCreate(const char* const* ifaces, size_t ifaces_length);

/**
 * \brief Frees resources created by the lookup object.
 *
 * Lookup object should no longer be used after this function is called!
 * Note that background query thread is stopped by this function.
 */
void hebiLookupRelease(HebiLookupPtr lookup);

/**
 * \brief sets the lookup request rate [Hz]
 *
 * \returns HebiStatusSuccess on success, HebiStatusInvalidArgument on negative or non-finite frequency
 */
HebiStatusCode hebiLookupSetLookupFrequencyHz(HebiLookupPtr lookup, double frequency);

/**
 * \brief gets the lookup request rate [Hz]
 */
double hebiLookupGetLookupFrequencyHz(HebiLookupPtr lookup);

/**
 * \brief Return a snapshot of the contents of the module registry -- i.e.,
 * which modules have been found by the lookup.
 *
 * \param lookup A valid HebiLookup object.
 */
HebiLookupEntryListPtr hebiCreateLookupEntryList(HebiLookupPtr lookup);

/**
 * Gets the number of entries in the lookup entry list.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 */
size_t hebiLookupEntryListGetSize(HebiLookupEntryListPtr lookup_list);

/**
 * Gets the name of the given entry in the lookup entry list. Must be a valid
 * index.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * Note - assumes ASCII string encoding.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 * \param index The entry index that is being queried.
 * \param buffer An allocated buffer of length 'length'
 * \param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character. This argument must not be NULL.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLookupEntryListGetName(HebiLookupEntryListPtr lookup_list, size_t index, char* buffer,
                                          size_t* length);

/**
 * Gets the family of the given entry in the lookup entry list. Must be a valid
 * index.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * Note - assumes ASCII string encoding.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 * \param index The entry index that is being queried.
 * \param buffer An allocated buffer of length 'length'.
 * \param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character. This argument must not be NULL.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLookupEntryListGetFamily(HebiLookupEntryListPtr lookup_list, size_t index, char* buffer,
                                            size_t* length);

/**
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 * \param index The entry index that is being queried.
 * \param mac_address A pointer to an allocated HebiMacAddress structure that
 * the function will update with the mac address of the given entry.
 *
 * \returns HebiStatusSuccess on success, HebiStatusInvalidArgument if the
 * mac_address parameter is null, or HebiStatusArgumentOutOfRange if there is
 * no entry with the given index.
 */
HebiStatusCode hebiLookupEntryListGetMacAddress(HebiLookupEntryListPtr lookup_list, size_t index,
                                                HebiMacAddress* mac_address);

/**
 * \brief Release resources for a given lookup entry list; list should not be
 * used after this call.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 */
void hebiLookupEntryListRelease(HebiLookupEntryListPtr lookup_list);

////////////////////////////////////////////////////////////////////////////////
/// Group API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates an "imitation" group with the specified number of modules.
 *
 * The imitation group is useful for testing the API, as it acts like a Group
 * would in most cases, but is not backed by hardware.  Commands that are
 * sent to the imitation group are returned as feedback, using the standard
 * feedback request methods.
 *
 * Note that standard groups are created through the HebiLookup objects.
 *
 * \param size The number of modules in the group.
 *
 * \returns An imitation group that returns commanded values as feedback.
 */
HebiGroupPtr hebiGroupCreateImitation(size_t size);

/**
 * \brief Create a group of modules with the given MAC addresses.
 *
 * If any given modules are not found, no group is created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param addresses An array of pointers to physical mac addresses of the given
 * modules. Length of the array must equal num_addresses. This param must not be
 * NULL, and each element of this list must not be NULL.
 * \param num_addresses Length of the addresses array of pointers (number of
 * pointers in the array, not cumulative size of objects they point to).
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromMacs(HebiLookupPtr lookup, const HebiMacAddress* const* addresses, size_t num_addresses,
                                     int32_t timeout_ms);

/**
 * \brief Create a group with modules matching the given names and families.
 *
 * If only one family is given, it is used for all modules.  Otherwise, number of
 * names and families must match. If any given modules are not found, no group is
 * created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param families The given families of the modules, as viewable in the HEBI
 * GUI. Must be a list of pointers to null-terminated strings. The number of
 * pointers must match the num_families parameter. Note that a single string
 * (with corresponding value of num_families == 1) will be used with each name
 * in the names list. This param must not be NULL, and each element of this list
 * must not be NULL.
 * \param num_families The number of pointers to null-terminated strings given
 * by the families parameter. Note that this must either be 1, or be equal to
 * num_names.
 * \param names The given names of the modules, as viewable in the HEBI GUI. Must
 * be a list of pointers to null-terminated strings. The number of pointers must
 * match the num_names parameter. This param must not be NULL, and each element
 * of this list must not be NULL.
 * \param num_names The number of pointers to null-terminated strings given
 * by the names parameter.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromNames(HebiLookupPtr lookup, const char* const* families, size_t num_families,
                                      const char* const* names, size_t num_names, int32_t timeout_ms);

/**
 * \brief Create a group with all modules known to the lookup with the given family.
 *
 * Group contains all modules with the given family, regardless of name.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param family The given family of the modules, as viewable in the HEBI GUI.
 * Must be a null-terminated string, and must not be NULL.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromFamily(HebiLookupPtr lookup, const char* family, int32_t timeout_ms);

/**
 * \brief Create a group with all modules connected to module with the given MAC
 * address.
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param address Pointer to a HebiMacAddress structure representing the
 * physical mac address of the given module (serves as unique id). Must not be
 * NULL.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateConnectedFromMac(HebiLookupPtr lookup, const HebiMacAddress* address, int32_t timeout_ms);

/**
 * \brief Create a group with all modules connected to module with the given name
 * and family
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param name The given name of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string, and must not be NULL.
 * \param family The given family of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string, and must not be NULL.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateConnectedFromName(HebiLookupPtr lookup, const char* family, const char* name,
                                              int32_t timeout_ms);

/**
 * \brief Returns the number of modules in a group.
 *
 * \param group The group to send this command to.
 *
 * \returns the number of modules in the group.
 */
size_t hebiGroupGetSize(HebiGroupPtr group);

/**
 * \brief Sends a command to the given group, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * Note: A non-HebiStatusSuccess return does not indicate a specific failure,
 * and may result from an error while sending or simply a timeout/dropped
 * response packet after a successful transmission.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns HebiStatusSuccess if an acknowledgement was successfully received (guaranteeing
 * the group received this command), or a failure code for an error otherwise.
 */
HebiStatusCode hebiGroupSendCommandWithAcknowledgement(HebiGroupPtr group, HebiGroupCommandPtr command,
                                                       int32_t timeout_ms);

/**
 * \brief Sends a command to the given group without requesting an
 * acknowledgement.
 *
 * Appropriate for high-frequency applications.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 *
 * \returns HebiStatusSuccess if the command was successfully sent, otherwise a failure code.
 */
HebiStatusCode hebiGroupSendCommand(HebiGroupPtr group, HebiGroupCommandPtr command);

/**
 * \brief Sets the command lifetime for the group, in milliseconds.
 *
 * The command lifetime is the duration for which a sent command remains active.
 * If the hardware does not receive further commands within the specified time
 * frame, all local controllers get deactivated. This is a safety feature to
 * mitigate the risk of accidents in case programs get interrupted in an unsafe
 * state, e.g., on program exceptions or during a network fault.
 *
 * Additionally, supporting hardware does not accept commands from any other
 * sources during the lifetime of a command. This mitigates the risk of other
 * users accidentally sending conflicting targets from, e.g., the GUI.
 *
 * \param group Which group the command lifetime is being set for.
 * \param lifetime_ms The number of milliseconds which the command 'lives' for.
 * Setting a value less than or equal to '0' disables command lifetime. When
 * disabled, the hardware will continue to execute the last sent command.
 * Setting a value above the accepted maximum will set the lockout to the
 * maximum value.
 *
 * \returns HebiStatusSuccess if command lifetime successfully set, or a failure code if
 * value was outside of accepted range (higher than supported maximum or negative).
 */
HebiStatusCode hebiGroupSetCommandLifetime(HebiGroupPtr group, int32_t lifetime_ms);

/**
 * \brief Returns the current command lifetime, in milliseconds.
 *
 * \param group Which group is being queried.
 *
 * \returns The current command lifetime, in milliseconds. A value of '0' indicates
 * that commands remain active until the next command is received.
 */
int32_t hebiGroupGetCommandLifetime(HebiGroupPtr group);

/**
 * \brief Sets the feedback request loop frequency (in Hz).
 *
 * The group is queried for feedback in a background thread at this frequency,
 * and any added callbacks are called from this background thread.
 *
 * \param group Which group this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns HebiStatusSuccess if feedback frequency successfully set, or a failure code if
 * value was outside of accepted range (higher than supported maximum, NaN or negative).
 */
HebiStatusCode hebiGroupSetFeedbackFrequencyHz(HebiGroupPtr group, float frequency);

/**
 * \brief Returns the current feedback request loop frequency (in Hz).
 *
 * \param group Which group is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiGroupGetFeedbackFrequencyHz(HebiGroupPtr group);

/**
 * \brief Add a function that is called whenever feedback is returned from the
 * group.
 *
 * \param group The group to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the group.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
HebiStatusCode hebiGroupRegisterFeedbackHandler(HebiGroupPtr group, GroupFeedbackHandlerFunction handler,
                                                void* user_data);

/**
 * \brief Removes all feedback handling functions from the queue to be called on
 * receipt of group feedback.
 *
 * \param group The group to which the handlers are attached.
 */
void hebiGroupClearFeedbackHandlers(HebiGroupPtr group);

/**
 * \brief Requests feedback from the group.
 *
 * Sends a background request to the modules in the group; if/when all modules
 * return feedback, any associated handler functions are called. This returned
 * feedback is also stored to be returned by the next call to
 * hebiGroupGetNextFeedback (any previously returned data is discarded).
 *
 * \param group The group to return feedback from.
 *
 * \returns HebiStatusSuccess if request was successfully sent, or a failure code if not
 * (i.e., connection error).
 */
HebiStatusCode hebiGroupSendFeedbackRequest(HebiGroupPtr group);

/**
 * \brief Returns the most recently stored feedback from a sent feedback
 * request, or returns the next one received (up to the requested timeout).
 *
 * Note that a feedback request can be sent either with the
 * hebiGroupSendFeedbackRequest function, or by setting a background feedback
 * frequency with hebiGroupSetFeedbackFrequencyHz.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param group The group to return feedback from.
 * \param feedback On success, the feedback read from the group are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for feedback.
 * For typical networks, '15' ms is a value that can be reasonably expected to
 * allow for a round trip transmission after the last 'send feedback request'
 * call.
 *
 * \returns HebiStatusSuccess if feedback was returned, or a failure code if not
 * (i.e., connection error or timeout waiting for response).
 */
HebiStatusCode hebiGroupGetNextFeedback(HebiGroupPtr group, HebiGroupFeedbackPtr feedback, int32_t timeout_ms);

/**
 * \brief Requests info from the group, and writes it to the provided info
 * object.
 *
 * Warning: if successful, other data in the provided 'Info' object is erased!
 *
 * \param group The group to send this command to.
 * \param info On success, the info read from the group is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns HebiStatusSuccess if info was received, or a failure code if not
 * (i.e., connection error or timeout waiting for response).
 */
HebiStatusCode hebiGroupRequestInfo(HebiGroupPtr group, HebiGroupInfoPtr info, int32_t timeout_ms);

/**
 * \brief Starts logging data to a file.
 *
 * Note: If a non null parameter is used for the returned string, and it is populated with a non null reference to a
 * string, it must be explicitly freed using @c hebiStringRelease(HebiStringPtr str)
 *
 * \param dir The relative or absolute path to the directory in which to log.
 * To use the current directory, pass in a null pointer
 * \param file The optional file name. If this is null, a name will be created using
 * the time at the moment which this function was invoked.
 * \param ret The optional pointer to a string reference. If this is null, this is ignored. Otherwise,
 * a reference to a string is populated with the path and name of the log file created. If this function does not
 * return HebiStatusSuccess and this parameter is not null, the value at this pointer is set to null.
 *
 * \returns HebiStatusSuccess if successfully started a log, a failure code otherwise.
 */
HebiStatusCode hebiGroupStartLog(HebiGroupPtr group, const char* dir, const char* file, HebiStringPtr* ret);

/**
 * \brief Stops logging data to a file.
 *
 * Note: This function allocates a log file structure on the heap, so make sure to release the pointer
 * returned by this function by calling @c hebiLogFileRelease(HebiLogFilePtr ptr)
 *
 * \param group The group that is logging.
 *
 * \returns a log file instance on success, otherwise a null pointer.
 */
HebiLogFilePtr hebiGroupStopLog(HebiGroupPtr group);

/**
 * \brief Release resources for a given group; group should not be used after
 * this call.
 *
 * \param group A valid HebiGroup object.
 */
void hebiGroupRelease(HebiGroupPtr group);

/**
 * \brief Creates a GroupCommand for a group with the specified number of
 * modules.
 *
 * \param size The number of modules in the group.
 *
 * \returns A pointer to a new GroupCommand object. This must be released
 * with \c hebiGroupCommandRelease(HebiGroupCommandPtr).
 */
HebiGroupCommandPtr hebiGroupCommandCreate(size_t size);

/**
 * \brief Return the number of modules in this group Command.
 *
 * \returns The number of module commands in this group command.
 */
size_t hebiGroupCommandGetSize(HebiGroupCommandPtr command);

/**
 * \brief Import gains from a file into a GroupCommand object.
 *
 * \param file A null-terminated string that gives the path/filename of the
 * gains XML file. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or a failure code if the file could
 * note be opened or read.
 * In particular:
 *  - if the file was successfully read, but the number of modules in the file
 *    is not equal to '1' or the size of the group, returns
 *    HebiStatusInvalidArgument.
 *  - if file is NULL, returns HebiStatusInvalidArgument.
 *  - for other parsing errors, may return HebiStatusFailure or
 *    HebiStatusInvalidArgument.
 */
HebiStatusCode hebiGroupCommandReadGains(HebiGroupCommandPtr command, const char* file);

/**
 * \brief Export gains from a GroupCommand object into a file.
 *
 * \param file A null-terminated string that gives the path/filename to save the
 * gains XML file at. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or a failure code if the file could
 * not be written to or an internal error occurs.
 */
HebiStatusCode hebiGroupCommandWriteGains(HebiGroupCommandPtr command, const char* file);

/**
 * \brief Import safety parameters from a file into a GroupCommand object.
 *
 * \param file A null-terminated string that gives the path/filename of the
 * safety parameters XML file. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or a failure code if the file could
 * note be opened or read.
 * In particular:
 *  - if the file was successfully read, but the number of modules in the file
 *    is not equal to '1' or the size of the group, returns
 *    HebiStatusInvalidArgument.
 *  - if file is NULL, returns HebiStatusInvalidArgument.
 *  - for other parsing errors, may return HebiStatusFailure or
 *    HebiStatusInvalidArgument.
 */
HebiStatusCode hebiGroupCommandReadSafetyParameters(HebiGroupCommandPtr command, const char* file);

/**
 * \brief Export safety parameters from a GroupCommand object into a file.
 *
 * \param file A null-terminated string that gives the path/filename to which
 * the safety parameters will be saved. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or a failure code if the file could
 * not be written to or an internal error occurs.
 */
HebiStatusCode hebiGroupCommandWriteSafetyParameters(HebiGroupCommandPtr command, const char* file);

/**
 * \brief Get an individual command for a particular module at index
 * \c module_index.
 *
 * \param module_index The index to retrieve the module command; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * \returns The command corresponding to the module at index \c module_index.
 */
HebiCommandPtr hebiGroupCommandGetModuleCommand(HebiGroupCommandPtr command, size_t module_index);

/**
 * \brief Clears the `dest` GroupCommand object, and copies all data from the
 * `src` GroupCommand object to `dest`.
 *
 * The GroupCommand objects should have identical state after this.
 *
 * \param dest The GroupCommandPtr to copy data into. The previous state of
 * this object will be cleared.
 * \param src The GroupCommandPtr to copy data from. This object will remain
 * unchanged.
 *
 * \return HebiStatusSuccess if the operation succeeds; HebiStatusInvalidArgument
 * if the sizes of the two messages do not match; HebiStatusError for any other
 * error.
 */
HebiStatusCode hebiGroupCommandCopy(HebiGroupCommandPtr dest, HebiGroupCommandPtr src);

/**
 * \brief Clears all data in the GroupCommand object.
 */
void hebiGroupCommandClear(HebiGroupCommandPtr command);

/**
 * \brief Frees resources created by the GroupCommand object.
 *
 * The GroupCommandPtr must not be used after this function is called.
 */
void hebiGroupCommandRelease(HebiGroupCommandPtr command);

/**
 * \brief Creates a GroupFeedback for a group with the specified number of
 * modules.
 *
 * \param size The number of modules in the group.
 *
 * \returns A pointer to a new GroupFeedback object. This must be released
 * with \c hebiGroupFeedbackRelease(HebiGroupFeedbackPtr).
 */
HebiGroupFeedbackPtr hebiGroupFeedbackCreate(size_t size);

/**
 * \brief Return the number of modules in this group Feedback.
 *
 * \returns The number of module feedbacks in this group feedback.
 */
size_t hebiGroupFeedbackGetSize(HebiGroupFeedbackPtr feedback);

/**
 * \brief Get an individual feedback for a particular module at index
 * \c module_index.
 *
 * \param module_index The index to retrieve the module feedback; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * \returns The feedback corresponding to the module at index \c module_index.
 */
HebiFeedbackPtr hebiGroupFeedbackGetModuleFeedback(HebiGroupFeedbackPtr feedback, size_t module_index);

/**
 * \brief Clears the `dest` GroupFeedback object, and copies all data from the
 * `src` GroupFeedback object to `dest`.
 *
 * The GroupFeedback objects should have identical state after this.
 *
 * \param dest The GroupFeedbackPtr to copy data into. The previous state of
 * this object will be cleared.
 * \param src The GroupFeedbackPtr to copy data from. This object will remain
 * unchanged.
 *
 * \return HebiStatusSuccess if the operation succeeds; HebiStatusInvalidArgument
 * if the sizes of the two messages do not match; HebiStatusError for any other
 * error.
 */
HebiStatusCode hebiGroupFeedbackCopy(HebiGroupFeedbackPtr dest, HebiGroupFeedbackPtr src);

/**
 * \brief Clears all data in the GroupFeedback object.
 */
void hebiGroupFeedbackClear(HebiGroupFeedbackPtr feedback);

/**
 * \brief Frees resources created by the GroupFeedback object.
 *
 * The GroupFeedbackPtr must not be used after this function is called.
 */
void hebiGroupFeedbackRelease(HebiGroupFeedbackPtr feedback);

/**
 * \brief Creates a GroupInfo for a group with the specified number of
 * modules.
 *
 * \param size The number of modules in the group.
 *
 * \returns A pointer to a new GroupInfo object. This must be released
 * with \c hebiGroupInfoRelease(HebiGroupInfoPtr).
 */
HebiGroupInfoPtr hebiGroupInfoCreate(size_t size);

/**
 * \brief Return the number of modules in this group Info.
 *
 * \returns The number of module infos in this group info.
 */
size_t hebiGroupInfoGetSize(HebiGroupInfoPtr info);

/**
 * \brief Export gains from a GroupInfo object into a file.
 *
 * \param file A null-terminated string that gives the path/filename to save the
 * gains XML file at. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or a failure code if the file could
 * not be written to or an internal error occurs.
 */
HebiStatusCode hebiGroupInfoWriteGains(HebiGroupInfoPtr info, const char* file);

/**
 * \brief Export safety parameters from a GroupInfo object into a file.
 *
 * \param file A null-terminated string that gives the path/filename to which
 * the safety parameters will be saved. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or a failure code if the file could
 * not be written to or an internal error occurs.
 */
HebiStatusCode hebiGroupInfoWriteSafetyParameters(HebiGroupInfoPtr info, const char* file);

/**
 * \brief Get an individual info for a particular module at index
 * \c module_index.
 *
 * \param module_index The index to retrieve the module info; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * \returns The info corresponding to the module at index \c module_index.
 */
HebiInfoPtr hebiGroupInfoGetModuleInfo(HebiGroupInfoPtr info, size_t module_index);

/**
 * \brief Clears the `dest` GroupInfo object, and copies all data from the
 * `src` GroupInfo object to `dest`.
 *
 * The GroupInfo objects should have identical state after this.
 *
 * \param dest The GroupInfoPtr to copy data into. The previous state of
 * this object will be cleared.
 * \param src The GroupInfoPtr to copy data from. This object will remain
 * unchanged.
 *
 * \return HebiStatusSuccess if the operation succeeds; HebiStatusInvalidArgument
 * if the sizes of the two messages do not match; HebiStatusError for any other
 * error.
 */
HebiStatusCode hebiGroupInfoCopy(HebiGroupInfoPtr dest, HebiGroupInfoPtr src);

/**
 * \brief Clears all data in the GroupInfo object.
 */
void hebiGroupInfoClear(HebiGroupInfoPtr info);

/**
 * \brief Frees resources created by the GroupInfo object.
 *
 * The GroupInfoPtr must not be used after this function is called.
 */
void hebiGroupInfoRelease(HebiGroupInfoPtr info);

////////////////////////////////////////////////////////////////////////////////
/// Command API
////////////////////////////////////////////////////////////////////////////////

/**
 * Retrieves the value and/or the length of the string field.
 *
 * If this string is not set in the Command, HebiStatusValueNotSet is
 * returned (regardless of any other arguments) and any pointer values are
 * unchanged.
 *
 * If 'length' is NULL, returns HebiStatusValueSuccess if this string value is
 * set (the value of 'buffer' is ignored in this case).
 *
 * If 'buffer' is not NULL, then 'length' should be non-NULL and set to the
 * number of characters that can be written to 'buffer'. If the specified string
 * is set, and it can fit into the provided buffer (including a null terminating
 * character), then the string is copied to 'buffer' and HebiStatusSuccess is
 * returned. If the buffer is not large enough to contain the string, then
 * HebiStatusBufferTooSmall is returned.
 *
 * If 'buffer' is NULL, returns HebiStatusValueSuccess if this string value is
 * set.
 *
 * If 'length' is not NULL (regardless of the state of 'buffer'), it is set to
 * the necessary size to hold the specified string value (including the null
 * terminating character).
 *
 * Note - assumes ASCII string encoding.
 */
HebiStatusCode hebiCommandGetString(HebiCommandPtr command, HebiCommandStringField field, char* buffer, size_t* length);

/**
 * Sets the given string to the value given in the buffer (if given). If any of
 * the provided pointers are null, the field is cleared.
 *
 * If not NULL, 'length' should be set to the length of the c-style string in
 * 'buffer', without including the terminating null character. The data in
 * 'buffer' does not need to be null terminated.
 *
 * Note - assumes ASCII string encoding.
 */
void hebiCommandSetString(HebiCommandPtr command, HebiCommandStringField field, const char* buffer,
                          const size_t* length);

/**
 * Retrieves a reference of the Command instance for direct access of fields
 */
void hebiCommandGetReference(HebiCommandPtr command, HebiCommandRef* ref);

/**
 * Retrieve the static metadata for all Command instances
 */
void hebiCommandGetMetadata(HebiCommandMetadata* metadata);

////////////////////////////////////////////////////////////////////////////////
/// Feedback API
////////////////////////////////////////////////////////////////////////////////


/**
 * Retrieves a reference of the Feedback instance for direct access of fields
 */
void hebiFeedbackGetReference(HebiFeedbackPtr feedback, HebiFeedbackRef* ref);

/**
 * Retrieve the static metadata for all Feedback instances
 */
void hebiFeedbackGetMetadata(HebiFeedbackMetadata* metadata);

////////////////////////////////////////////////////////////////////////////////
/// Info API
////////////////////////////////////////////////////////////////////////////////

/**
 * Retrieves the value and/or the length of the string field.
 *
 * If this string is not set in the Info, HebiStatusValueNotSet is
 * returned (regardless of any other arguments) and any pointer values are
 * unchanged.
 *
 * If 'length' is NULL, returns HebiStatusValueSuccess if this string value is
 * set (the value of 'buffer' is ignored in this case).
 *
 * If 'buffer' is not NULL, then 'length' should be non-NULL and set to the
 * number of characters that can be written to 'buffer'. If the specified string
 * is set, and it can fit into the provided buffer (including a null terminating
 * character), then the string is copied to 'buffer' and HebiStatusSuccess is
 * returned. If the buffer is not large enough to contain the string, then
 * HebiStatusBufferTooSmall is returned.
 *
 * If 'buffer' is NULL, returns HebiStatusValueSuccess if this string value is
 * set.
 *
 * If 'length' is not NULL (regardless of the state of 'buffer'), it is set to
 * the necessary size to hold the specified string value (including the null
 * terminating character).
 *
 * Note - assumes ASCII string encoding.
 */
HebiStatusCode hebiInfoGetString(HebiInfoPtr info, HebiInfoStringField field, char* buffer, size_t* length);

/**
 * Retrieves a reference of the Info instance for direct access of fields
 */
void hebiInfoGetReference(HebiInfoPtr info, HebiInfoRef* ref);

/**
 * Retrieve the static metadata for all Info instances
 */
void hebiInfoGetMetadata(HebiInfoMetadata* metadata);

////////////////////////////////////////////////////////////////////////////////
/// RobotModel API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates a one-dof joint about the specified axis.
 *
 * \param joint_type What the axis of motion for this joint is. The coordinate
 * frame is relative to the previous robot model element this is attached to.
 *
 * \returns NULL if the joint could not be created; otherwise, pointer
 * to newly allocated joint.  Must either be added to a robot model object or
 * released via hebiRobotModelElementRelease(HebiRobotModelElementPtr).
 */
HebiRobotModelElementPtr hebiRobotModelElementCreateJoint(HebiJointType joint_type);

/**
 * \brief Creates a rigid body defining static transforms to the given outputs.
 *
 * \param com Matrix (4x4 tranform) for the center of mass location, including
 * the frame which the inertia tensor is given in. Must not be NULL.
 * \param inertia Vector (6 elements, Ixx, Iyy, Izz, Ixy, Ixz, Iyz) of the
 * inertia tensor, in the frame given by the COM. Must not be NULL.
 * \param mass Mass (in kg) of the rigid body.
 * \param num_outputs The number of available outputs.
 * \param outputs Matrices (list of n 4x4 transforms) of the transforms from
 * input to the available outputs of the body. Must not be NULL unless
 * num_outputs is 0.
 * \param ordering How the data in `com` and `outputs` should be serialized.
 *
 * NOTE: currently, num_outputs must be 1.
 *
 * \returns NULL if the rigid body could not be created; otherwise, pointer to
 * newly allocated body.  Must either be added to a robot model object or
 * released via 'hebiRobotModelElementRelease'.
 */
HebiRobotModelElementPtr hebiRobotModelElementCreateRigidBody(const double* com, const double* inertia, double mass,
                                                              size_t num_outputs, const double* outputs,
                                                              HebiMatrixOrdering ordering);

/**
 * \brief Creates a robot model element corresponding to a standard HEBI end
 * effector, or a custom end effector defining static transforms to the given
 * outputs.
 *
 * \param end_effector_type The support end effector type from the HEBI
 * component library, as given by the HebiEndEffectorType enum. If "Custom" is
 * not given, then mass, center of mass, inertia, and output frame are ignored.
 *
 * \param com Matrix (4x4 tranform) for the center of mass location, including
 * the frame which the inertia tensor is given in. Only used if
 * end_effector_type is Custom; defaults to (0,0,0) if NULL.
 * \param inertia Vector (6 elements, Ixx, Iyy, Izz, Ixy, Ixz, Iyz) of the
 * inertia tensor, in the frame given by the COM. Only used if end_effector_type
 * is Custom; defaults to point mass (all elements 0) if NULL.
 * \param mass Mass (in kg) of the rigid body. Only used if end_effector_type is
 * Custom.
 * \param output_frame A matrix (4x4 transform) from the input to the available
 * frames of the end effector. Only used if end_effector_type is Custom;
 * defaults to identity if NULL.
 * \param ordering How the data in `com` and `output_frame` should be serialized.
 *
 * \returns NULL if the rigid body could not be created; otherwise, pointer to
 * newly allocated body.  Must either be added to a robot model object or
 * released via 'hebiRobotModelElementRelease'.
 */
HebiRobotModelElementPtr hebiRobotModelElementCreateEndEffector(HebiEndEffectorType end_effector_type,
                                                                const double* com, const double* inertia,
                                                                double mass, const double* output_frame,
                                                                HebiMatrixOrdering ordering);

/**
 * \brief Creates a robot model element corresponding to a standard HEBI actuator.
 *
 * \param actuator_type The supported actuator type from the HEBI component
 * library, as given by the HebiActuatorType enum.
 *
 * \returns NULL if the actuator could not be created; otherwise, pointer to
 * newly allocated element.  Must either be added to a robot model object or
 * released via 'hebiRobotModelElementRelease'.
 */
HebiRobotModelElementPtr hebiRobotModelElementCreateActuator(HebiActuatorType actuator_type);

/**
 * \brief Creates a rigid body, including mass and static transforms,
 * corresponding to a standard HEBI bracket.
 *
 * \param bracket_type The supported bracket type from the HEBI component
 * library, as given by the HebiBracketType enum.
 *
 * \returns NULL if the rigid body could not be created; otherwise, pointer to
 * newly allocated body.  Must either be added to a robot model object or
 * released via 'hebiRobotModelElementRelease'.
 */
HebiRobotModelElementPtr hebiRobotModelElementCreateBracket(HebiBracketType bracket_type);

/**
 * \brief Creates a rigid body, including mass and static transforms,
 * corresponding to a standard HEBI link.
 *
 * \param link_type The supported link type from the HEBI component
 * library, as given by the HebiLinkType enum.
 * \param input_type The supported link input style, as given by the
 * HebiLinkInputType enum.
 * \param output_type The supported link output style, as given by the
 * HebiLinkOutputType enum.
 * \param extension The z-axis to z-axis input to output distance, as defined
 * at docs.hebi.us.
 * \param twist Rotation (in radians) about the extension axis, as defined at
 * docs.hebi.us.
 *
 * \returns NULL if the rigid body could not be created; otherwise, pointer to
 * newly allocated body.  Must either be added to a robot model object or
 * released via 'hebiRobotModelElementRelease'.
 */
HebiRobotModelElementPtr hebiRobotModelElementCreateLink(HebiLinkType link_type, HebiLinkInputType input_type,
                                                         HebiLinkOutputType output_type, double extension,
                                                         double twist);

/**
 * \brief Frees resources created by this element.
 *
 * Note: Only do this if element has not been added to a robot model object!
 * Once added, the robot model object manages the element's resources.
 *
 * The element should no longer be used after this function is called.
 *
 * \param A valid robot model element object which has not been added to a
 * robot_model object.
 */
void hebiRobotModelElementRelease(HebiRobotModelElementPtr element);

/**
 * \brief Import robot model from a file into a RobotModel object.
 *
 * \param file A null-terminated string that gives the path/filename of the
 * robot model XML file (HRDF). Must not be NULL. The format of this file is
 * documented at docs.hebi.us.
 *
 * \returns A HebiRobotModelPtr on success, or NULL if the file could not be
 * opened or read.  In the case that NULL is returned, an error message is set
 * that is retrievable via hebiRobotModelGetImportError.  This saved error
 * message is thread-safe, and is available until hebiRobotModelImport is called
 * again on this thread.  In the case that a non-NULL object is returned, this
 * must be released with 'hebiRobotModelRelease' when no longer needed.
 */
HebiRobotModelPtr hebiRobotModelImport(const char* file);

/**
 * \brief Import robot model from a buffer into a RobotModel object.
 *
 * \param buffer A buffer of the text content of an robot model XML file (HRDF).
 * Must not be NULL. The format of this file is documented at docs.hebi.us.
 * \param buffer_size The length of the above buffer.  The buffer does not need
 * to be null terminated, and as such this buffer_size should not include any
 * null terminating characters.
 *
 * \returns A HebiRobotModelPtr on success, or NULL if the file could not be
 * parsed.  In the case that NULL is returned, an error message is set that is
 * retrievable via hebiRobotModelGetImportError.  This saved error message is
 * thread-safe, and is available until hebiRobotModelImport is called again on
 * this thread.  In the case that a non-NULL object is returned, this must be
 * released with 'hebiRobotModelRelease' when no longer needed.
 */
HebiRobotModelPtr hebiRobotModelImportBuffer(const char* buffer, size_t buffer_size);

/**
 * \brief Retrieve any error string from the last call to hebiRobotModelImport.
 * This must be called on the same thread as the call to hebiRobotModelImport.
 *
 * \returns A null-terminated ASCII character array with a human readable error
 * message describing the failure.  This array is valid until the next call to
 * 'hebiRobotModelImport' on this thread; accessing the array after that call
 * is undefined behavior.
 */
const char* hebiRobotModelGetImportError();

/**
 * \brief Retrieve the number of warnings corresponding to the last call to
 * hebiRobotModelImport. This must be called on the same thread as the call to
 * hebiRobotModelImport.
 *
 * \returns The number of recoverable warnings encountered when importing the
 * file.
 */
size_t hebiRobotModelGetImportWarningCount();

/**
 * \brief Retrieve the 'ith' warning string from the last call to
 * hebiRobotModelImport. This must be called on the same thread as the call to
 * hebiRobotModelImport.
 *
 * \param warning_index The 0-based index of the warning to retrieve.  Must be
 * in the range [0, `hebiRobotModelGetImportWarningCount`), or NULL will be
 * returned.
 *
 * \returns A null-terminated ASCII character array with a human readable error
 * message describing the failure.  This array is valid until the next call to
 * 'hebiRobotModelImport' on this thread; accessing the array after that call
 * is undefined behavior.
 */
const char* hebiRobotModelGetImportWarning(size_t warning_index);

/**
 * \brief Creates an object to hold a robot model (tree topology). This
 * structure has a single output available at the origin.
 *
 * RobotModel object created by this function must be released with
 * 'hebiRobotModelRelease' when no longer needed.
 */
HebiRobotModelPtr hebiRobotModelCreate(void);

/**
 * \brief Sets the fixed transform from the origin to the input of the first
 * added model element.
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param transform A 4x4 homogeneous transform. Must not be null, and must be
 * a valid transform.
 * \param ordering How the data in `transform` should be serialized.
 *
 * \returns HebiStatusSuccess on success, or HebiStatusInvalidArgument if
 * transform is null or invalid.
 */
HebiStatusCode hebiRobotModelSetBaseFrame(HebiRobotModelPtr robot_model, const double* transform,
                                          HebiMatrixOrdering ordering);

/**
 * \brief Retreives the fixed transform from the origin to the input of the
 * first added model element.
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param transform A allocated 16 element array of doubles; this is filled
 * in by the function with the 4x4 homogeneous transform. Must not be NULL.
 * \param ordering How the data in `transform` should be serialized.
 *
 * \returns HebiStatusSuccess on success, or HebiStatusInvalidArgument if
 * transform is null.
 */
HebiStatusCode hebiRobotModelGetBaseFrame(HebiRobotModelPtr robot_model, double* transform,
                                          HebiMatrixOrdering ordering);

/**
 * \brief Return the number of frames in the forward kinematic tree of the robot
 * model.
 *
 * Note that this depends on the type of frame requested -- for center of mass
 * frames, there is one per added rigid body (that was not combined with
 * another); for output frames, there is one per output per element.  For end
 * effectors, this is the total number of outputs on the leaves of the kinematic
 * tree.
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 */
size_t hebiRobotModelGetNumberOfFrames(HebiRobotModelPtr robot_model, HebiFrameType frame_type);

/**
 * \brief Returns the number of settable degrees of freedom in the kinematic
 * tree. (This is equal to the number of joints added).
 *
 * \param robot_model A valid HEBI RobotModel object.
 */
size_t hebiRobotModelGetNumberOfDoFs(HebiRobotModelPtr robot_model);

/**
 * \brief Returns the number of elements added to the kinematic
 * tree.
 *
 * \param robot_model A valid HEBI RobotModel object.
 */
size_t hebiRobotModelGetNumberOfElements(HebiRobotModelPtr robot_model);

/**
 * \brief Retrieves metadata about an element in the robot model. This
 * metadata includes what type of element this is, and per-element-type
 * information.
 *
 * There is one "metadata" object for each element in the robot model.
 *
 * \param model The robot model pointer to query.
 * \param index The index of the element in the robot model; this corresponds
 * to the "robot model element" frame with the same index.
 * \param output This should be a HebiRobotModelElementMetadata structure, and
 * cannot be NULL.
 *
 * \return On success, HebiStatusSuccess will be returned. If "model" or "output"
 * is NULL, then HebiStatusInvalidArgument will be returned.  If the index does
 * not exist in this robot model, then HebiStatusArgumentOutOfRange will be
 * returned.
 */
HebiStatusCode hebiRobotModelGetElementMetadata(HebiRobotModelPtr model, size_t index, HebiRobotModelElementMetadata* output);

/**
 * \brief Add an element to a parent element connected to a robot model object.
 *
 * After the addition, the robot model object manages the resources of the added
 * element.  On failure, the passed in 'new_element' argument is properly
 * disposed by this function.
 *
 * The added element is assumed to connect to an available output on a element
 * that has already been attached to the kinematic tree. That element should be
 * passed in as 'existing_element', and the index of the requested output on
 * that element should be given as 'output_index'.
 *
 * To attach the initial element to the robot model object, use NULL for the
 * 'existing_element' argument.
 *
 * NOTE: currently, only a single output is supported for each element (e.g., a
 * kinematic chain), and so the 'existing_element' and 'output_index' parameters
 * are not checked; this will be changed in an upcoming minor release so do not
 * rely on this behavior!
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param existing_element The parent element which the element is added to (or
 * NULL to add the initial element to the tree).
 * \param output_index The index of the requested output on the parent element
 * on which to attach this element.
 * \param new_element The kinematic element which is added to the tree. Must not
 * be NULL.
 * \param combine Whether or not to combine this with the output frame where
 * this is being attached to, essentially "hiding" this existing output frame
 * from returned frames and replacing it with the output from this.  Body masses
 * and inertias are also combined. '1' means combine; '0' means do not combine.
 *
 * \returns HebiStatusSuccess on success, otherwise HebiStatusFailure (e.g., the
 * parent body's requested output is invalid or already occupied) or
 * HebiStatusInvalidArgument (e.g., 'new_element' is NULL).
 */
HebiStatusCode hebiRobotModelAdd(HebiRobotModelPtr robot_model, HebiRobotModelElementPtr existing_element,
                                 size_t output_index, HebiRobotModelElementPtr new_element);

/**
 * \brief Generates the transforms for the forward kinematics of the given
 * robot model.
 *
 * The order of the returned frames is in a depth-first tree. As an example,
 * assume an element A has one output, to which element B is connected to.
 * Element B has two outputs; C is attached to the first output and E is
 * attached to the second output. Element D is attached to the only output of
 * element C:
 *
 * (BASE) A - B(1) - C - D
 *           (2)
 *            |
 *            E
 *
 * For center of mass frames, the returned frames would be A-B-C-D-E.
 *
 * For output frames, the returned frames would be A-B(1)-C-D-B(2)-E.
 *
 * For end effector frames, the returned frames are the outputs of the leaf
 * nodes; here the output of D and E.
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 * \param positions A vector of joint positions/angles (in SI units of meters or
 * radians) equal in length to the number of DoFs of the robot model. Must not
 * be NULL.
 * \param frames An allocated (16 x number of frames) array of doubles; this is
 * filled in by the function with the 4x4 homogeneous transform of each frame.
 * Note that the number of frames depends on the frame type! Must not be NULL.
 * \param ordering How the data in `frames` should be serialized.
 *
 * \returns HebiStatusSuccess on success, or HebiStatusInvalidArgument if
 * positions or frames are NULL.
 */
HebiStatusCode hebiRobotModelGetForwardKinematics(HebiRobotModelPtr robot_model, HebiFrameType frame_type,
                                                  const double* positions, double* frames, HebiMatrixOrdering ordering);

/**
 * \brief Generates the jacobian for each frame in the given kinematic tree.
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 * \param positions A vector of joint positions/angles (in SI units of meters or
 * radians) equal in length to the number of DoFs of the robot model. Must not
 * be NULL.
 * \param jacobians An allocated (6 x number of dofs x number of frames) array
 * of doubles; this is filled in by the function with the 6 x number of dofs
 * jacobian for each frame.  Note that the number of frames depends on the frame
 * type! Must not be NULL.
 * \param ordering How the data in `jacobians` should be serialized.
 *
 * \returns HebiStatusSuccess on success, or HebiStatusInvalidArgument if
 * positions or jacobians are NULL.
 */
HebiStatusCode hebiRobotModelGetJacobians(HebiRobotModelPtr robot_model, HebiFrameType frame_type,
                                          const double* positions, double* jacobians, HebiMatrixOrdering ordering);

/**
 * \brief Fill in the masses vector with the mass of each body with mass in the
 * kinematic tree, reported in a depth-first ordering.
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param masses An allocated array of doubles, with length equal to the return
 * value of hebiRobotModelGetNumberOfFrames with argument
 * HebiFrameTypeCenterOfMass. Must not be NULL.
 *
 * \returns HebiStatusSuccess on success, or HebiStatusInvalidArgument if
 * masses is NULL.
 */
HebiStatusCode hebiRobotModelGetMasses(HebiRobotModelPtr robot_model, double* masses);


/**
 * \brief returns table of information about the shape of kinematic tree
 *
 * \param robot_model A valid HEBI RobotModel object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum 
 * (index of table corresponds to index of forward kinematic frames of the specified type).
 * \param table An allocated (number of frames for frame_type) array of HebiRobotModelElementTopology objects;
 *  this is filled by the function.
 *
 *  \returns HebiStatusSuccess on success or HebiStatusInvalidArgument if
 *  robot_model or table is null.
 */
HebiStatusCode hebiRobotModelGetTreeTopology(HebiRobotModelPtr robot_model, HebiFrameType frame_type, HebiRobotModelElementTopology* table);


/**
 * \brief Frees resources created by this robot model object.
 *
 * RobotModel object should no longer be used after this function is called!
 *
 * \param robot_model A valid HEBI RobotModel object.
 */
void hebiRobotModelRelease(HebiRobotModelPtr robot_model);

////////////////////////////////////////////////////////////////////////////////
/// Inverse Kinematics API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates an IK (inverse kinematics) object that allows for solving for
 * joint angles/positions given objectives and constraints.
 *
 * This optimization is completed using a specified forward kinematics object.
 * The objectives and constraints are stored with this object.
 */
HebiIKPtr hebiIKCreate(void);

/**
 * \brief Add an objective that optimizes for the end effector output frame
 * origin to be at the given (x, y, z) point.
 *
 * If an end effector position objective already exists, this will replace it.
 *
 * \param weight The weight of this objective relative to any other objective
 * functions (this objective is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param x The desired x position of the end effector frame. 'NaN' can be
 * passed to ignore this variable.
 * \param y The desired y position of the end effector frame. 'NaN' can be
 * passed to ignore this variable.
 * \param z The desired z position of the end effector frame. 'NaN' can be
 * passed to ignore this variable.
 *
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g. incompatible with
 * existing objectives, or all components are set to 'NaN')
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorPosition(HebiIKPtr ik, double weight, size_t end_effector_index, double x,
                                                     double y, double z);

/**
 * \brief Add an objective that optimizes for the end effector output frame
 * orientation to be given by the 3x3 rotation matrix. Note that this is
 * incompatible with the end effector tip axis objective.
 *
 * If an end effector orientation objective already exists, this will replace
 * it.
 *
 * \param weight The weight of this objective relative to any other objective
 * functions (this objective is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param matrix The desired orientation of the end effector frame, as a 3x3
 * rotation matrix. Must not be NULL.
 * \param ordering How the data in `matrix` should be serialized.
 *
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g. incompatible with
 * existing objectives, or rotation matrix is invalid or null.)
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorSO3(HebiIKPtr ik, double weight, size_t end_effector_index,
                                                const double* matrix, HebiMatrixOrdering ordering);

/**
 * \brief Add an objective that points the end effector's z axis in a given
 * direction.  Note that this is incompatible with the end effector S03
 * orientation objective.
 *
 * If an end effector orientation objective already exists, this will replace
 * it.
 *
 * \param weight The weight of this objective relative to any other objective
 * functions (this objective is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param x The desired end effector z axis, projected onto the x axis.
 * \param y The desired end effector z axis, projected onto the y axis.
 * \param z The desired end effector z axis, projected onto the z axis.
 *
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g. incompatible with
 * existing objectives, or rotation matrix is invalid.)
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorTipAxis(HebiIKPtr ik, double weight, size_t end_effector_index, double x,
                                                    double y, double z);

/**
 * \brief Define joint angle constraints.
 *
 * NaN or +/- infinity can be set on particular joints to ignore joint limits.
 * Currently, the joint limit constraints are two-sided, which means that any
 * joint must either have both min/max set to NaN/inf, or have neither.
 *
 * \param weight The weight of this constraint relative to any other objective
 * functions (this constraint is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param num_joints The number of elements in the min_positions and
 * max_positions arrays.
 * \param min_positions An array with the minimum joint limit for each joint, or
 * NaN or inf if unlimited. Must have num_joints elements, and must not be NULL.
 * \param max_positions An array with the maximum joint limit for each joint, or
 * NaN or inf if unlimited. Must have num_joints elements, and must not be NULL.
 *
 * \return HebiStatusSuccess on success; otherwise a failure code.
 */
HebiStatusCode hebiIKAddConstraintJointAngles(HebiIKPtr ik, double weight, size_t num_joints,
                                              const double* min_positions, const double* max_positions);

/**
 * \brief Add a custom objective function to be minimized by the IK solver.
 *
 * The given callback is called at each iteration of the local optimization, and
 * is expected to fill in each entry of the "errors" vector.
 *
 * \param weight The weight of this constraint relative to any other objective
 * functions (this constraint is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param num_errors The number of independent error values that this objective
 * returns.
 * \param err_fnc The callback function which is used to compute the errors. The
 * arguments are:
 *   user_data - The pointer passed in when adding a custom objective; this can
 *     be anything (even NULL), and is designed to allow the user to easily
 *     access information in the callback.
 *   num_positions - the number of elements in the 'positions' array
 *   positions - an array of the joint positions at this point in the
 *     optimization; of length 'num_positions'
 *   errors - an array of error values which must be filled in by the callback;
 *     of length 'num_errors'.
 */
HebiStatusCode hebiIKAddObjectiveCustom(HebiIKPtr ik, double weight, size_t num_errors,
                                        void (*err_fnc)(void* user_data, size_t num_positions, const double* positions,
                                                        double* errors),
                                        void* user_data);

/**
 * \brief Clears the objectives and constraints from this IK object, along
 * with any modifications to the default algorithm parameters.
 */
void hebiIKClearAll(HebiIKPtr ik);

/**
 * \brief Solves for an inverse kinematics solution that moves the end effector
 * to a given point.
 *
 * Note: multiple "hebiIKSolve" calls can be made using the same IK object.
 *
 * \param ik A valid HEBI IK object.
 * \param model A valid HEBI RobotModel object.
 * \param initial_positions The seed positions/angles (in SI units of meters or
 * radians) to start the IK search from; equal in length to the number of DoFs
 * of the kinematic tree. Must not be NULL.
 * \param ik_solution Allocated array of doubles equal in length to the
 * number of DoFs of the kinematic tree; the function will fill in this array
 * with the IK solution (in SI units of meters or radians). Must not be NULL.
 * \param result_info Reserved for future use (will enable more information
 * about output of optimization such as success/failure, function error, etc).
 * This should currently be set to NULL.
 *
 * \return HebiStatusSuccess on success, other values on failure (e.g., no objectives given or
 * dimension mismatch between kinematics object and stored objectives).
 */
HebiStatusCode hebiIKSolve(HebiIKPtr ik, HebiRobotModelPtr model, const double* initial_positions, double* ik_solution,
                           void* result_info);

/**
 * \brief Frees resources created by this inverse kinematics object.
 *
 * IK object should no longer be used after this function is called!
 *
 * \param ik A valid HEBI inverse kinematics object.
 */
void hebiIKRelease(HebiIKPtr ik);

////////////////////////////////////////////////////////////////////////////////
/// Trajectory API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates a HebiTrajectory object for a single joint using the given
 * parameters; this must be released with hebiTrajectoryRelease after use.
 *
 * \param num_waypoints The number of waypoints.
 * \param positions A vector of waypoints for this joint; should be
 * num_waypoints in length. Any elements that are NAN will be considered free
 * parameters, and will be set by the function. Values of +/-infinity are not
 * allowed. Must not be NULL.
 * \param velocities An optional vector of velocity constraints at the
 * corresponding waypoints; should either be NULL or num_waypoints in length.
 * Any elements that are NAN will be considered free parameters, and will be set
 * by the function.  Values of +/-infinity are not allowed.
 * \param accelerations An optional vector of acceleration constraints at the
 * corresponding waypoints; should either be NULL or num_waypoints in length.
 * Any elements that are NAN will be considered free parameters, and will be set
 * by the function.  Values of +/-infinity are not allowed.
 * \param time_vector A vector of times to reach each waypoint; this must
 * be defined (not NULL, and not NAN for any element). The first element must
 * be zero.
 *
 * \returns A HebiTrajectory object if there were no errors, and the trajectory
 * has been created. A NULL value indicates that there was an error, but does
 * not specify any details about the error at this time.
 */
HebiTrajectoryPtr hebiTrajectoryCreateUnconstrainedQp(size_t num_waypoints, const double* positions,
                                                      const double* velocities, const double* accelerations,
                                                      const double* time_vector);

/**
 * \brief Frees resources created by this trajectory.
 *
 * Trajectory should no longer be used after this function is called!
 */
void hebiTrajectoryRelease(HebiTrajectoryPtr trajectory);

/**
 * \brief Returns the length of this trajectory (in seconds).
 */
double hebiTrajectoryGetDuration(HebiTrajectoryPtr trajectory);

/**
 * \brief Gets the value of the trajectory at a given time.
 *
 * \param trajectory A HebiTrajectory object
 * \param time The time within the trajectory (from the beginning to the end of
 * the trajectory's time vector) at which to query.
 * \param position Filled in with the position at the given time, as defined by
 * this trajectory. Must not be null.
 * \param velocity Filled in with the velocity at the given time, as defined by
 * this trajectory. Must not be null.
 * \param acceleration Filled in with the acceleration at the given time, as
 * defined by this trajectory. Must not be null.
 *
 * \returns HebiStatusSuccess on success, otherwise an error status.
 */
HebiStatusCode hebiTrajectoryGetState(HebiTrajectoryPtr trajectory, double time, double* position, double* velocity,
                                      double* acceleration);

////////////////////////////////////////////////////////////////////////////////
/// Logging API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Releases a log file instance
 */
void hebiLogFileRelease(HebiLogFilePtr log_file);

/**
 * \brief Copy the path and name of the log file into a buffer.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * \param buffer Pointer to a buffer into which the string will be written. This can be null, in which case this
 * function will write the size of the string (including null character) into length.
 * \param length Pointer to the length of the input buffer. This parameter must not be null, or this function
 * will return HebiStatusInvalidArgument
 * \return HebiStatusSuccess on success, HebiStatusBufferTooSmall if the value referenced by length is smaller than
 *         the string (including the character), or HebiStatusInvalidArgument if length pointer is null
 */
HebiStatusCode hebiLogFileGetFileName(HebiLogFilePtr log_file, char* buffer, size_t* length);

/**
 * \brief Opens an existing log file
 *
 * Note: It is up to the user to check if the returned pointer is null. If the file does not exist,
 * or if the file is not a valid log file, this function returns null.
 *
 * If this function returns a pointer, you must call @c hebiLogFileRelease(HebiLogFilePtr)
 * to release the allocated memory.
 *
 * \param file the directory and path of the file to open.  Must not be NULL.
 * \return a pointer to the file;  null if the file could not be opened
 */
HebiLogFilePtr hebiLogFileOpen(const char* file);

/**
 * \brief Retrieve the number of modules in the group represented by an opened log file
 *
 * \return The number of modules in the group
 */
size_t hebiLogFileGetNumberOfModules(HebiLogFilePtr log_file);

/**
 * \brief Retrieve the next group feedback from the opened log file
 *
 * \param feedback the feedback object into which the contents will be copied
 * \return HebiStatusSuccess on success, otherwise HebiStatusFailure
 */
HebiStatusCode hebiLogFileGetNextFeedback(HebiLogFilePtr log_file, HebiGroupFeedbackPtr field);

////////////////////////////////////////////////////////////////////////////////
/// String Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Copy the string into a buffer
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null terminating character.
 *
 * \param buffer Pointer to a buffer into which the string will be written. This can be null, in which case this
 * function will write the size of the string (including null character) into length.
 * \param length Pointer to the length of the input buffer. This parameter must not be null, or this function
 * will return HebiStatusInvalidArgument
 * \return HebiStatusSuccess on success, HebiStatusBufferTooSmall if the value referenced by length is smaller than
 * the string (including the character), or HebiStatusInvalidArgument if length pointer is null
 */
HebiStatusCode hebiStringGetString(HebiStringPtr str, char* buffer, size_t* length);

/**
 * \brief Releases a string instance
 */
void hebiStringRelease(HebiStringPtr str);

////////////////////////////////////////////////////////////////////////////////
/// Misc Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Get the string corresponding to the last error from any safety parameters function.
 * 
 * Note: This function returns the last error from the current thread.
 * 
 * \return a NULL terminated string which must not be deallocated
 */
const char* hebiSafetyParametersGetLastError(void);

/**
 * \brief Get the version of the library
 *
 * All parameters must not be NULL.
 *
 * \return HebiStatusSuccess on success, otherwise HebiStatusInvalidArgument if
 * a parameter is NULL.
 */
HebiStatusCode hebiGetLibraryVersion(int32_t* major, int32_t* minor, int32_t* revision);

/**
 * \brief Frees all resources created by the library.  Note: any calls to the
 * HEBI library functions after this will result in undefined behavior!
 */
void hebiCleanup(void);

#ifdef __cplusplus
} // extern "C"
#endif
