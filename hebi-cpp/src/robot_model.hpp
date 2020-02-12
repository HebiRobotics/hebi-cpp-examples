#pragma once

#include "hebi.h"

#include <memory>
#include <vector>

#include "Eigen/Eigen"
#include "util.hpp"

using namespace Eigen;

namespace hebi {
namespace robot_model {

using Matrix4dVector = std::vector<Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
using MatrixXdVector = std::vector<MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
// The result of an IK operation.  More fields will be added to this structure
// in future API releases.
struct IKResult {
  HebiStatusCode result; // Success or failure
};

class RobotModel;

class Objective {
  friend RobotModel;

public:
  virtual ~Objective() {}

protected:
  virtual HebiStatusCode addObjective(HebiIKPtr ik) const = 0;
};

class EndEffectorPositionObjective final : public Objective {
public:
  EndEffectorPositionObjective(const Eigen::Vector3d&);
  EndEffectorPositionObjective(double weight, const Eigen::Vector3d&);

private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  double _weight, _x, _y, _z;
};

class EndEffectorSO3Objective final : public Objective {
public:
  EndEffectorSO3Objective(const Eigen::Matrix3d&);
  EndEffectorSO3Objective(double weight, const Eigen::Matrix3d&);

private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  double _weight;
  const double _matrix[9];
};

class EndEffectorTipAxisObjective final : public Objective {
public:
  EndEffectorTipAxisObjective(const Eigen::Vector3d&);
  EndEffectorTipAxisObjective(double weight, const Eigen::Vector3d&);

private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  double _weight, _x, _y, _z;
};

class JointLimitConstraint final : public Objective {
public:
  JointLimitConstraint(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions);
  JointLimitConstraint(double weight, const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions);

private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override;
  double _weight;
  Eigen::VectorXd _min_positions;
  Eigen::VectorXd _max_positions;

public:
  // Allow Eigen member variables:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * INTERNAL
 * \brief C-style callback wrapper to call into CustomObjective class; this should only
 * be used by the CustomObjective class itself.
 *
 * This is forward declared here so it can be used in the CustomObjective class
 * below.
 */
template<size_t T>
inline void custom_objective_callback_wrapper(void* user_data, size_t num_positions, const double* positions,
                                              double* errors);

/**
 * \brief Allows you to add a custom objective function.
 *
 * To use, you must implement and pass in a std::function object that takes a
 * vector of positions and an array of error values that you should fill in:
 *
 * \code{.cpp}
 * std::function<void(const std::vector<double>&, std::array<double, N>&)>;
 * \endcode
 *
 * This function is called at each step of the optimization.
 *
 * Note that the template parameter N is the number of independent errors that
 * your objective function sets.
 *
 * Example usage, using a lambda function for the callback.  Note that this toy
 * example optimizes for joint angles that sum to 2:
 *
 * \code{.cpp}
 * Eigen::VectorXd initial_joint_angles(group->size());
 * Eigen::VectorXd ik_result_joint_angles(group->size());
 *
 * model->solveIK(
 *   initial_joint_angles,
 *   ik_result_joint_angles,
 *   robot_model::CustomObjective<1>(
 *     [](const std::vector<double> positions, std::array<double, 1>& errors)
 *     {
 *       // Add up all the joint angles
 *       double sum = 0;
 *       for (auto p : positions)
 *         sum += p;
 *       // This objective prefers joint angles that sum to '2'
 *       errors[0] = 2 - sum;
 *     }
 *   )
 * );
 * \endcode
 */
template<size_t N>
class CustomObjective final : public Objective {
public:
  // This function is called with the following parameters:
  // const std::vector<double>& positions
  // std::array<double, N>& errors (NOTE: FILL THIS IN VIA THE CALLBACK FUNCTION)
  using ObjectiveCallback = std::function<void(const std::vector<double>&, std::array<double, N>&)>;

  CustomObjective(ObjectiveCallback error_function) : _weight(1.0f), _callback(error_function) {}
  CustomObjective(double weight, ObjectiveCallback error_function) : _weight(weight), _callback(error_function) {}

  // Note -- internal function to be called only from
  // custom_objective_callback_wrapper.
  void callCallback(void*, size_t num_positions, const double* positions, double* errors) const {
    // Note -- the data here is copied to/from the C-style arrays.  This isn't
    // optimally efficient, but allows us to use type-checked C++ vectors and
    // arrays.  For performance critical applications, this could be modified
    // to support user callbacks using the raw C-style arrays directly.

    // Process data into C++ structures.
    std::vector<double> positions_array(num_positions);
    for (size_t i = 0; i < num_positions; ++i)
      positions_array[i] = positions[i];
    // Note -- std::array is not guaranteed to be layout-compatible with a
    // C-style array, even for POD, so we must copy here it we want the type
    // safety of std::array.
    std::array<double, N> errors_array;
    for (size_t i = 0; i < N; ++i)
      errors_array[i] = errors[i];

    _callback(positions_array, errors_array);

    for (size_t i = 0; i < N; ++i)
      errors[i] = errors_array[i];
  }

private:
  HebiStatusCode addObjective(HebiIKPtr ik) const override {
    return hebiIKAddObjectiveCustom(ik, _weight, N, &custom_objective_callback_wrapper<N>,
                                    const_cast<CustomObjective*>(this));
  }
  double _weight;
  ObjectiveCallback _callback;
};

/**
 * INTERNAL
 * \brief C-style callback wrapper to call into CustomObjective class; this should only
 * be used by the CustomObjective class itself.
 */
template<size_t T>
inline void custom_objective_callback_wrapper(void* user_data, size_t num_positions, const double* positions,
                                              double* errors) {
  reinterpret_cast<CustomObjective<T>*>(user_data)->callCallback(user_data, num_positions, positions, errors);
}

class ActuatorMetadata;
class BracketMetadata;
class JointMetadata;
class LinkMetadata;
class RigidBodyMetadata;

enum class ElementType {
  Other = HebiRobotModelElementTypeOther,
  Actuator = HebiRobotModelElementTypeActuator,
  Bracket = HebiRobotModelElementTypeBracket,
  Joint = HebiRobotModelElementTypeJoint,
  Link = HebiRobotModelElementTypeLink,
  RigidBody = HebiRobotModelElementTypeRigidBody,
  EndEffector = HebiRobotModelElementTypeEndEffector
};

enum class FrameType {
  CenterOfMass = HebiFrameTypeCenterOfMass,
  Output = HebiFrameTypeOutput,
  EndEffector = HebiFrameTypeEndEffector,
  Input = HebiFrameTypeInput
};

enum class JointType {
  RotationX = HebiJointTypeRotationX,
  RotationY = HebiJointTypeRotationY,
  RotationZ = HebiJointTypeRotationZ,
  TranslationX = HebiJointTypeTranslationX,
  TranslationY = HebiJointTypeTranslationY,
  TranslationZ = HebiJointTypeTranslationZ
};

enum class ActuatorType {
  X5_1 = HebiActuatorTypeX5_1,
  X5_4 = HebiActuatorTypeX5_4,
  X5_9 = HebiActuatorTypeX5_9,
  X8_3 = HebiActuatorTypeX8_3,
  X8_9 = HebiActuatorTypeX8_9,
  X8_16 = HebiActuatorTypeX8_16,
  R8_3 = HebiActuatorTypeR8_3,
  R8_9 = HebiActuatorTypeR8_9,
  R8_16 = HebiActuatorTypeR8_16
};

enum class LinkType {
  X5 = HebiLinkTypeX5,
  X8 = HebiLinkTypeR8
};

enum class LinkInputType {
  RightAngle = HebiLinkInputTypeRightAngle,
  Inline = HebiLinkInputTypeInline
};

enum class LinkOutputType {
  RightAngle = HebiLinkOutputTypeRightAngle,
  Inline = HebiLinkOutputTypeInline
};

enum class BracketType {
  X5LightLeft = HebiBracketTypeX5LightLeft,
  X5LightRight = HebiBracketTypeX5LightRight,
  X5HeavyLeftInside = HebiBracketTypeX5HeavyLeftInside,
  X5HeavyLeftOutside = HebiBracketTypeX5HeavyLeftOutside,
  X5HeavyRightInside = HebiBracketTypeX5HeavyRightInside,
  X5HeavyRightOutside = HebiBracketTypeX5HeavyRightOutside,
  R8LightLeft = HebiBracketTypeR8LightLeft,
  R8LightRight = HebiBracketTypeR8LightRight,
  R8HeavyLeftInside = HebiBracketTypeR8HeavyLeftInside,
  R8HeavyLeftOutside = HebiBracketTypeR8HeavyLeftOutside,
  R8HeavyRightInside = HebiBracketTypeR8HeavyRightInside,
  R8HeavyRightOutside = HebiBracketTypeR8HeavyRightOutside
};

enum class EndEffectorType {
  Custom = HebiEndEffectorTypeCustom,
  X5Parallel = HebiEndEffectorTypeX5Parallel,
  R8Parallel = HebiEndEffectorTypeR8Parallel
};


/**
 * @brief Base class for all metadata. Do not instantiate directly.
 */
class MetadataBase {
  friend class RobotModel;
public:
  MetadataBase() = default;
  MetadataBase(const MetadataBase&) = delete;
  MetadataBase& operator=(const MetadataBase&) = delete;
  MetadataBase(MetadataBase&&) = default;
  MetadataBase& operator=(MetadataBase&&) = default;

  ElementType elementType() const {
    return static_cast<ElementType>(metadata_.element_type_);
  }

  /**
   * @brief View the actuator specific fields, if this metadata describes an actuator.
   * 
   * Note: This will return @c nullptr if this metadata does not describe an actuator.
   */
  const ActuatorMetadata* asActuator() const {
    if (elementType() == ElementType::Actuator) {
      return reinterpret_cast<const ActuatorMetadata*>(this);
    }
    return nullptr;
  }

  /**
   * @brief View the bracket specific fields, if this metadata describes a bracket.
   * 
   * Note: This will return @c nullptr if this metadata does not describe a bracket.
   */
  const BracketMetadata* asBracket() const {
    if (elementType() == ElementType::Bracket) {
      return reinterpret_cast<const BracketMetadata*>(this);
    }
    return nullptr;
  }

  /**
   * @brief View the joint specific fields, if this metadata describes a joint.
   * 
   * Note: This will return @c nullptr if this metadata does not describe a joint.
   */
  const JointMetadata* asJoint() const {
    if (elementType() == ElementType::Joint) {
      return reinterpret_cast<const JointMetadata*>(this);
    }
    return nullptr;
  }

  /**
   * @brief View the link specific fields, if this metadata describes a link.
   * 
   * Note: This will return @c nullptr if this metadata does not describe a link.
   */
  const LinkMetadata* asLink() const {
    if (elementType() == ElementType::Link) {
      return reinterpret_cast<const LinkMetadata*>(this);
    }
    return nullptr;
  }

  /**
   * @brief View the rigid body specific fields, if this metadata describes a rigid body.
   * 
   * Note: This will return @c nullptr if this metadata does not describe a rigid body.
   */
  const RigidBodyMetadata* asRigidBody() const {
    if (elementType() == ElementType::RigidBody) {
      return reinterpret_cast<const RigidBodyMetadata*>(this);
    }
    return nullptr;
  }

protected:
  // Raw data can only be accessed by the sub classes
  const HebiRobotModelElementMetadata& metadata() const { return metadata_; }
private:
  HebiRobotModelElementMetadata metadata_;
};

/**
 * @brief Actuator specific view of an element in a RobotModel instance
 */
class ActuatorMetadata : public MetadataBase {
public:
  /**
   * @brief Specifies the particular actuator model
   */
  ActuatorType actuatorType() const { return static_cast<ActuatorType>(metadata().actuator_type_); }
};

/**
 * @brief Bracket specific view of an element in a RobotModel instance
 */
class BracketMetadata : public MetadataBase {
public:
  /**
   * @brief Specifies the particular bracket type
   */
  BracketType bracketType() const { return static_cast<BracketType>(metadata().bracket_type_); }
};

/**
 * @brief Joint specific view of an element in a RobotModel instance
 */
class JointMetadata : public MetadataBase {
public:
  /**
   * @brief Specifies the particular joint type
   */
  JointType jointType() const { return static_cast<JointType>(metadata().joint_type_); }
};

/**
 * @brief Link specific view of an element in a RobotModel instance
 */
class LinkMetadata : public MetadataBase {
public:
  /**
   * @brief Specifies the particular link type
   */
  LinkType linkType() const { return static_cast<LinkType>(metadata().link_type_); }

  /**
   * @brief The extension of the link (_e.g._, the value specified in invocation of RobotModel::addLink)
   */
  float extension() const { return metadata().extension_; }

  /**
   * @brief The twist of the link (_e.g._, the value specified in invocation of RobotModel::addLink)
   */
  float twist() const { return metadata().twist_; }
};

/**
 * @brief Rigid Body specific view of an element in a RobotModel instance
 */
class RigidBodyMetadata : public MetadataBase {};

/**
 * \brief Represents a chain or tree of robot elements (rigid bodies and
 * joints).
 *
 * (Currently, only chains of elements are fully supported)
 */
class RobotModel final {
  friend Objective;

private:
  /**
   * C-style robot model object
   */
  HebiRobotModelPtr const internal_;

  /**
   * Internal function for resolving variadic template (stops recursion and
   * detects invalid arguments).
   */
  template<typename T>
  HebiStatusCode addObjectives(HebiIKPtr ik, const T& objective) const {
    static_assert(std::is_convertible<T*, Objective*>::value,
                  "Must pass arguments of base type hebi::robot_model::Objective to the variable args of solveIK!");
    return static_cast<const Objective*>(&objective)->addObjective(ik);
  }

  /**
   * Internal function for resolving variadic template.
   */
  template<typename T, typename... Args>
  HebiStatusCode addObjectives(HebiIKPtr ik, const T& objective, Args... args) const {
    static_assert(std::is_convertible<T*, Objective*>::value,
                  "Must pass arguments of base type hebi::robot_model::Objective to the variable args of solveIK!");
    auto res = static_cast<const Objective*>(&objective)->addObjective(ik);
    if (res != HebiStatusSuccess)
      return res;
    return addObjectives(ik, args...);
  }

  /**
   * Internal function for adding robot model elements
   */
  bool tryAdd(HebiRobotModelElementPtr element);

  /**
   * Internal function for constructing a RobotModel object from a C API
   * pointer.
   */
  RobotModel(HebiRobotModelPtr);

public:

  using ActuatorType [[deprecated ("Replace with hebi::robot_model::ActuatorType")]] = robot_model::ActuatorType;
  using BracketType [[deprecated ("Replace with hebi::robot_model::BracketType")]] = robot_model::BracketType;
  using LinkType [[deprecated ("Replace with hebi::robot_model::LinkType")]] = robot_model::LinkType;

  /**
   * \brief Creates a robot model object with no bodies and an identity base
   * frame.
   */
  RobotModel();

  /**
   * \brief Creates a robot model object with no bodies and an identity base
   * frame.
   */
  static std::unique_ptr<RobotModel> loadHRDF(const std::string& file);

  /**
   * \brief Destructor cleans up robot model object, including all managed
   * elements.
   */
  ~RobotModel() noexcept;

  /**
   * \brief Set the transform from a world coordinate system to the input
   * of the root element in this robot model object. Defaults to an identity
   * 4x4 matrix.
   *
   * The world coordinate system is used for all position, vector, and
   * transformation matrix parameters in the member functions.
   *
   * \param base_frame The 4x4 homogeneous transform from the world frame to
   * the root kinematic body frame.
   */
  void setBaseFrame(const Eigen::Matrix4d& base_frame);

  /**
   * \brief Returns the transform from the world coordinate system to the root
   * kinematic body, as set by the setBaseFrame function.
   */
  Eigen::Matrix4d getBaseFrame() const;

  /**
   * \brief Return the number of frames in the forward kinematics.
   *
   * Note that this depends on the type of frame requested -- for center of mass
   * frames, there is one per added body; for output frames, there is one per
   * output per body.
   *
   * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
   */
  size_t getFrameCount(FrameType frame_type) const;

  /**
   * \brief Return the number of frames in the forward kinematics.
   *
   * \deprecated Replaced by getFrameCount(FrameType) accepting C++ enum
   */
  [[deprecated ("Replaced by hebi::robot_model::RobotModel::getFrameCount(hebi::robot_model::FrameType)")]]
  size_t getFrameCount(HebiFrameType frame_type) const { return getFrameCount(static_cast<FrameType>(frame_type)); }

  /**
   * \brief Returns the number of settable degrees of freedom in the kinematic
   * tree. (This is equal to the number of actuators added).
   */
  size_t getDoFCount() const;

  /**
   * \brief Adds a rigid body with the specified properties to the robot
   * model.
   *
   * \param com 4x4 matrix of the homogeneous transform to the center of mass
   * location, relative to the input frame of the element. Note that this
   * frame is also the frame the inertia tensor is given in.
   * \param inertia The 6 element representation (Ixx, Iyy, Izz, Ixy, Ixz,
   * Iyz) of the inertia tensor, in the frame given by the COM.
   * \param output 4x4 matrix of the homogeneous transform to the output frame,
   * relative to the input frame of the element.
   * \param mass The mass of this element.
   */
  bool addRigidBody(const Eigen::Matrix4d& com, const Eigen::VectorXd& inertia, double mass,
                    const Eigen::Matrix4d& output);

  /**
   * \brief Adds a degree of freedom about the specified axis.
   *
   * This does not represent an element with size or mass, but only a
   * connection between two other elements about a particular axis.
   *
   * \param joint_type The axis of rotation or translation about which this
   * joint allows motion.
   */
  bool addJoint(JointType joint_type);

  /**
   * \brief Adds a degree of freedom about the specified axis.
   *
   * \deprecated Replaced by addJoint(JointType) accepting C++ enum
   */
  [[deprecated ("Replaced by hebi::robot_model::RobotModel::addJoint(hebi::robot_model::JointType)")]]
  bool addJoint(HebiJointType joint_type) { return addJoint(static_cast<JointType>(joint_type)); }

  /**
   * \brief Add an element to the robot model with the kinematics/dynamics of
   * an X5 actuator.
   *
   * \param actuator_type The type of actuator to add.
   */
  bool addActuator(robot_model::ActuatorType actuator_type);

  /**
   * \brief Add an element to the robot model with the kinematics/dynamics of
   * a link between two actuators.
   *
   * \param link_type The type of link between the actuators, e.g. a tube link
   * between two X5 or X8 actuators.
   * \param extension The center-to-center distance between the actuator
   * rotational axes.
   * \param twist The rotation (in radians) between the actuator axes of
   * rotation. Note that a 0 radian rotation will result in a z-axis offset
   * between the two actuators, and a pi radian rotation will result in the
   * actuator interfaces to this tube being in the same plane, but the
   * rotational axes being anti-parallel.
   * \param input_type The style of input adapter on the link.
   * \param output_type The style of input adapter on the link.
   */
  bool addLink(robot_model::LinkType link_type, double extension, double twist,
               LinkInputType input_type = LinkInputType::RightAngle,
               LinkOutputType output_type = LinkOutputType::RightAngle);

  /**
   * \brief Add an element to the robot model with the kinematics/dynamics of
   * a bracket between two actuators.
   *
   * \param bracket_type The type of bracket to add.
   */
  bool addBracket(robot_model::BracketType bracket_type);

  /**
   * \brief Add an end effector element to the robot model.
   * 
   * For a "custom" type end effector, indentity transforms and
   * zero mass and inertia parameters are used.
   *
   * \param end_effector_type The type of end_effector to add.
   */
  bool addEndEffector(EndEffectorType end_effector_type);

  /**
   * \brief Generates the forward kinematics for the given robot model.
   *
   * See getFK for details.
   */
  void getForwardKinematics(FrameType, const Eigen::VectorXd& positions, Matrix4dVector& frames) const;

  /**
   * \brief Generates the forward kinematics for the given robot model.
   *
   * \deprecated Replaced by getForwardKinematics accepting C++ enum
   */
  [[deprecated ("Change first argument form HebiFrameType to hebi::robot_model::FrameType")]]
  void getForwardKinematics(HebiFrameType frame_type, const Eigen::VectorXd& positions, Matrix4dVector& frames) const {
    getForwardKinematics(static_cast<FrameType>(frame_type), positions, frames);
  }

  /**
   * \brief Generates the forward kinematics for the given robot model.
   *
   * The order of the returned frames is in a depth-first tree. As an example,
   * assume a body A has one output, to which body B is connected to. Body B has
   * two outputs; actuator C is attached to the first output and actuator E is
   * attached to the second output.  Body D is attached to the only output of
   * actuator C:
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
   * \param frame_type Which type of frame to consider -- see FrameType enum.
   * \param positions A vector of joint positions/angles (in SI units of meters or
   * radians) equal in length to the number of DoFs of the kinematic tree.
   * \param frames An array of 4x4 transforms; this is resized as necessary
   * in the function and filled in with the 4x4 homogeneous transform of each
   * frame. Note that the number of frames depends on the frame type.
   */
  void getFK(FrameType, const Eigen::VectorXd& positions, Matrix4dVector& frames) const;

  /**
   * \brief Generates the forward kinematics for the given robot model.
   *
   * \deprecated Replaced by getFK accepting C++ enum
   */
  [[deprecated ("Change first argument form HebiFrameType to hebi::robot_model::FrameType")]]
  void getFK(HebiFrameType frame_type, const Eigen::VectorXd& positions, Matrix4dVector& frames) const {
    getFK(static_cast<FrameType>(frame_type), positions, frames);
  }

  /**
   * \brief Generates the forward kinematics to the end effector (leaf node)
   * frame(s).
   *
   * Note -- for center of mass frames, this is one per leaf node; for output
   * frames, this is one per output per leaf node, in depth first order.
   *
   * This overload is for kinematic chains that only have a single leaf node
   * frame.
   *
   * (Currently, kinematic trees are not fully supported -- only kinematic
   * chains -- and so there are not other overloads for this function).
   *
   * \param positions A vector of joint positions/angles (in SI units of meters or
   * radians) equal in length to the number of DoFs of the kinematic tree.
   * \param transform A 4x4 transform that is resized as necessary in the
   * function and filled in with the homogeneous transform to the end
   * effector frame.
   */
  void getEndEffector(const Eigen::VectorXd& positions, Eigen::Matrix4d& transform) const;

  /**
   * \brief Solves for an inverse kinematics solution given a set of
   * objectives.
   *
   * See solveIK for details.
   */
  template<typename... Args>
  IKResult solveInverseKinematics(const Eigen::VectorXd& initial_positions, Eigen::VectorXd& result,
                                  Args... args) const {
    return solveIK(initial_positions, result, args...);
  }

  /**
   * \brief Solves for an inverse kinematics solution given a set of
   * objectives.
   *
   * \param initial_positions The seed positions/angles (in SI units of meters
   * or radians) to start the IK search from; equal in length to the number of
   * DoFs of the kinematic tree.
   * \param result A vector equal in length to the number of DoFs of the
   * kinematic tree; this will be filled in with the IK solution (in SI units
   * of meters or radians), and resized as necessary.
   * \param objectives A variable number of objectives used to define the IK
   * search (e.g., target end effector positions, etc). Each argument must
   * have a base class of Objective.
   */
  template<typename... Args>
  IKResult solveIK(const Eigen::VectorXd& initial_positions, Eigen::VectorXd& result, Args... objectives) const {
    // Create a HEBI C library IK object
    auto ik = hebiIKCreate();

    // (Try) to add objectives to the IK object
    IKResult res;
    res.result = addObjectives(ik, objectives...);
    if (res.result != HebiStatusSuccess) {
      hebiIKRelease(ik);
      return res;
    }

    // Transfer/initialize from Eigen to C arrays
    auto positions_array = new double[initial_positions.size()];
    {
      Map<Eigen::VectorXd> tmp(positions_array, initial_positions.size());
      tmp = initial_positions;
    }
    auto result_array = new double[initial_positions.size()];

    // Call into C library to solve
    res.result = hebiIKSolve(ik, internal_, positions_array, result_array, nullptr);

    // Transfer/cleanup from C arrays to Eigen
    delete[] positions_array;
    {
      Map<Eigen::VectorXd> tmp(result_array, initial_positions.size());
      result = tmp;
    }
    delete[] result_array;

    hebiIKRelease(ik);

    return res;
  }

  /**
   * \brief Generates the Jacobian for each frame in the given kinematic tree.
   *
   * See getJ for details.
   */
  void getJacobians(FrameType, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const;
  
  /**
   * \brief Generates the Jacobian for each frame in the given kinematic tree.
   *
   * \deprecated Replaced by getJacobians accepting C++ enum
   */
  [[deprecated]]
  void getJacobians(HebiFrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const {
    getJacobians(static_cast<FrameType>(frame_type), positions, jacobians);
  }

  /**
   * \brief Generates the Jacobian for each frame in the given kinematic tree.
   *
   * \param frame_type Which type of frame to consider -- see FrameType
   * enum.
   * \param positions A vector of joint positions/angles (in SI units of
   * meters or radians) equal in length to the number of DoFs of the kinematic
   * tree.
   * \param jacobians A vector (length equal to the number of frames) of
   * matrices; each matrix is a (6 x number of dofs) jacobian matrix for the
   * corresponding frame of reference on the robot.  This vector is resized as
   * necessary inside this function.
   */
  void getJ(FrameType, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const;

  /**
   * \brief Generates the Jacobian for each frame in the given kinematic tree.
   *
   * \deprecated Replaced by getJ accepting C++ enum
   */
  [[deprecated ("Change first argument form HebiFrameType to hebi::robot_model::FrameType")]]
  void getJ(HebiFrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const {
    getJ(static_cast<FrameType>(frame_type), positions, jacobians);
  }

  /**
   * \brief Generates the Jacobian for the end effector (leaf node) frames(s).
   *
   * See getJEndEffector for details.
   */
  void getJacobianEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const;
  /**
   * \brief Generates the Jacobian for the end effector (leaf node) frames(s).
   *
   * Note -- for center of mass frames, this is one per leaf node; for output
   * frames, this is one per output per leaf node, in depth first order.
   *
   * This overload is for kinematic chains that only have a single leaf node
   * frame.
   *
   * (Currently, kinematic trees are not fully supported -- only kinematic
   * chains -- and so there are not other overloads for this function).
   *
   * \param positions A vector of joint positions/angles (in SI units of
   * meters or radians) equal in length to the number of DoFs of the kinematic
   * tree.
   * \param jacobian A (6 x number of dofs) jacobian matrix for the
   * corresponding end effector frame of reference on the robot.  This vector
   * is resized as necessary inside this function.
   */
  void getJEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const;

  /**
   * \brief Returns the mass of each rigid body (or combination of rigid
   * bodies) in the robot model.
   *
   * \param masses A vector which is filled with the masses in the robot
   * model.  This vector is resized as necessary inside this function (it is
   * set to have length equal to the number of com frames).
   */
  void getMasses(Eigen::VectorXd& masses) const;

  /**
   * @brief Returns the metadata of each component of the robot model.
   */
  void getMetadata(std::vector<MetadataBase>& metadata) const;
private:
  /**
   * Disable copy and move constructors and assignment operators
   */
  HEBI_DISABLE_COPY_MOVE(RobotModel)
};

} // namespace robot_model
} // namespace hebi
