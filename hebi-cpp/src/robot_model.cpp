#include "robot_model.hpp"

#include <iostream>

namespace hebi {
namespace robot_model {

////////////////////////// Objectives

EndEffectorPositionObjective::EndEffectorPositionObjective(const Eigen::Vector3d& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

EndEffectorPositionObjective::EndEffectorPositionObjective(double weight, const Eigen::Vector3d& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

HebiStatusCode EndEffectorPositionObjective::addObjective(HebiIKPtr ik) const {
  return hebiIKAddObjectiveEndEffectorPosition(ik, static_cast<float>(_weight), 0, _x, _y, _z);
}

// clang-format off
EndEffectorSO3Objective::EndEffectorSO3Objective(const Eigen::Matrix3d& matrix)
  : _weight(1.0f), _matrix{
    matrix(0,0), matrix(1,0), matrix(2,0),
    matrix(0,1), matrix(1,1), matrix(2,1),
    matrix(0,2), matrix(1,2), matrix(2,2)}
{ }

EndEffectorSO3Objective::EndEffectorSO3Objective(double weight, const Eigen::Matrix3d& matrix)
  : _weight(weight), _matrix{
    matrix(0,0), matrix(1,0), matrix(2,0),
    matrix(0,1), matrix(1,1), matrix(2,1),
    matrix(0,2), matrix(1,2), matrix(2,2)}
{ }
// clang-format on

HebiStatusCode EndEffectorSO3Objective::addObjective(HebiIKPtr ik) const {
  return hebiIKAddObjectiveEndEffectorSO3(ik, _weight, 0, _matrix, HebiMatrixOrderingColumnMajor);
}

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(const Eigen::Vector3d& obj)
  : _weight(1.0f), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

EndEffectorTipAxisObjective::EndEffectorTipAxisObjective(double weight, const Eigen::Vector3d& obj)
  : _weight(weight), _x(obj[0]), _y(obj[1]), _z(obj[2]) {}

HebiStatusCode EndEffectorTipAxisObjective::addObjective(HebiIKPtr ik) const {
  return hebiIKAddObjectiveEndEffectorTipAxis(ik, _weight, 0, _x, _y, _z);
}

JointLimitConstraint::JointLimitConstraint(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions)
  : _weight(1.0f), _min_positions(min_positions), _max_positions(max_positions) {}

JointLimitConstraint::JointLimitConstraint(double weight, const Eigen::VectorXd& min_positions,
                                           const Eigen::VectorXd& max_positions)
  : _weight(weight), _min_positions(min_positions), _max_positions(max_positions) {}

HebiStatusCode JointLimitConstraint::addObjective(HebiIKPtr ik) const {
  if (_min_positions.size() != _max_positions.size())
    return HebiStatusInvalidArgument;

  int num_joints = _min_positions.size();

  auto min_positions_array = new double[num_joints];
  {
    Map<Eigen::VectorXd> tmp(min_positions_array, num_joints);
    tmp = _min_positions;
  }
  auto max_positions_array = new double[num_joints];
  {
    Map<Eigen::VectorXd> tmp(max_positions_array, num_joints);
    tmp = _max_positions;
  }

  auto res = hebiIKAddConstraintJointAngles(ik, _weight, num_joints, min_positions_array, max_positions_array);

  delete[] min_positions_array;
  delete[] max_positions_array;

  return res;
}

////////////////////////// RobotModel

bool RobotModel::tryAdd(HebiRobotModelElementPtr element) {
  HebiStatusCode res = hebiRobotModelAdd(internal_, nullptr, 0, element);
  if (res == HebiStatusFailure) {
    hebiRobotModelElementRelease(element);
    return false;
  }
  return true;
}

RobotModel::RobotModel(HebiRobotModelPtr internal) : internal_(internal) {}

RobotModel::RobotModel() : internal_(hebiRobotModelCreate()) {}

std::unique_ptr<RobotModel> RobotModel::loadHRDF(const std::string& file) {
  HebiRobotModelPtr internal = hebiRobotModelImport(file.c_str());
  // nullptr return means an import error; print error and return empty
  // unique_ptr.
  if (internal == nullptr) {
    std::cerr << "HRDF Error: " << hebiRobotModelGetImportError() << std::endl;
    return nullptr;
  }

  // Display any relevant warnings.
  size_t num_warnings = hebiRobotModelGetImportWarningCount();
  for (size_t i = 0; i < num_warnings; ++i)
    std::cerr << "HRDF Warning: " << hebiRobotModelGetImportWarning(i) << std::endl;

  // Create/return the robot model
  return std::unique_ptr<RobotModel>(new RobotModel(internal));
}

RobotModel::~RobotModel() noexcept { hebiRobotModelRelease(internal_); }

void RobotModel::setBaseFrame(const Eigen::Matrix4d& base_frame) {
  // Put data into an array
  double transform[16];
  Map<Matrix<double, 4, 4>> tmp(transform);
  tmp = base_frame;
  hebiRobotModelSetBaseFrame(internal_, transform, HebiMatrixOrderingColumnMajor);
}

Eigen::Matrix4d RobotModel::getBaseFrame() const {
  // Get the data into an array
  double transform[16];
  hebiRobotModelGetBaseFrame(internal_, transform, HebiMatrixOrderingColumnMajor);

  // Copy out data
  Map<const Matrix<double, 4, 4>> tmp(transform);
  Eigen::Matrix4d res;
  res = tmp;
  return res;
}

size_t RobotModel::getFrameCount(FrameType frame_type) const {
  return hebiRobotModelGetNumberOfFrames(internal_, static_cast<HebiFrameType>(frame_type));
}

size_t RobotModel::getDoFCount() const { return hebiRobotModelGetNumberOfDoFs(internal_); }

// TODO: handle trees/etc by passing in parent object here, and output index
bool RobotModel::addRigidBody(const Eigen::Matrix4d& com, const Eigen::VectorXd& inertia, double mass,
                              const Eigen::Matrix4d& output) {
  if (inertia.size() != 6)
    return false;

  // Allocate double arrays for C interop:
  double com_array[16];
  double inertia_array[6];
  double output_array[16];

  // Convert the data:
  {
    Map<Matrix<double, 4, 4>> tmp(com_array);
    tmp = com;
  }
  {
    Map<Eigen::VectorXd> tmp(inertia_array, 6);
    tmp = inertia;
  }
  {
    Map<Matrix<double, 4, 4>> tmp(output_array);
    tmp = output;
  }

  HebiRobotModelElementPtr body = hebiRobotModelElementCreateRigidBody(com_array, inertia_array, mass, 1, output_array, HebiMatrixOrderingColumnMajor);

  return tryAdd(body);
}

// TODO: handle trees/etc by passing in parent object here, and output index
bool RobotModel::addJoint(JointType joint_type) {
  return tryAdd(hebiRobotModelElementCreateJoint(static_cast<HebiJointType>(joint_type)));
}

bool RobotModel::addActuator(robot_model::ActuatorType actuator_type) {
  HebiRobotModelElementPtr element = hebiRobotModelElementCreateActuator(static_cast<HebiActuatorType>(actuator_type));
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

bool RobotModel::addLink(robot_model::LinkType link_type, double extension, double twist,
                         LinkInputType input_type, LinkOutputType output_type) {
  HebiRobotModelElementPtr element =
    hebiRobotModelElementCreateLink(static_cast<HebiLinkType>(link_type),
                                    static_cast<HebiLinkInputType>(input_type),
                                    static_cast<HebiLinkOutputType>(output_type),
                                    extension, twist);
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

bool RobotModel::addBracket(robot_model::BracketType bracket_type) {
  HebiRobotModelElementPtr element = hebiRobotModelElementCreateBracket(static_cast<HebiBracketType>(bracket_type));
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

bool RobotModel::addEndEffector(EndEffectorType end_effector_type) {
  auto element = hebiRobotModelElementCreateEndEffector(static_cast<HebiEndEffectorType>(end_effector_type), nullptr,
                                                        nullptr, 0, nullptr, HebiMatrixOrderingColumnMajor);
  if (element == nullptr)
    return false;
  return tryAdd(element);
}

void RobotModel::getForwardKinematics(FrameType frame_type, const Eigen::VectorXd& positions,
                                      Matrix4dVector& frames) const {
  getFK(frame_type, positions, frames);
}

void RobotModel::getFK(FrameType frame_type, const Eigen::VectorXd& positions, Matrix4dVector& frames) const {
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }
  size_t num_frames = getFrameCount(frame_type);
  auto frame_array = new double[16 * num_frames];
  // Get data from C API
  hebiRobotModelGetForwardKinematics(internal_, static_cast<HebiFrameType>(frame_type), positions_array, frame_array, HebiMatrixOrderingColumnMajor);
  delete[] positions_array;
  // Copy into vector of matrices passed in
  frames.resize(num_frames);
  for (size_t i = 0; i < num_frames; ++i) {
    Map<Matrix<double, 4, 4>> tmp(frame_array + i * 16);
    frames[i] = tmp;
  }
  delete[] frame_array;
}

void RobotModel::getEndEffector(const Eigen::VectorXd& positions, Eigen::Matrix4d& transform) const {
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }

  double transform_array[16];
  hebiRobotModelGetForwardKinematics(internal_, HebiFrameTypeEndEffector, positions_array, transform_array, HebiMatrixOrderingColumnMajor);
  delete[] positions_array;
  {
    Map<Matrix<double, 4, 4>> tmp(transform_array);
    transform = tmp;
  }
}

void RobotModel::getJacobians(FrameType frame_type, const Eigen::VectorXd& positions,
                              MatrixXdVector& jacobians) const {
  getJ(frame_type, positions, jacobians);
}
void RobotModel::getJ(FrameType frame_type, const Eigen::VectorXd& positions, MatrixXdVector& jacobians) const {
  // Put data into an array
  auto positions_array = new double[positions.size()];
  {
    Map<Eigen::VectorXd> tmp(positions_array, positions.size());
    tmp = positions;
  }

  size_t num_frames = getFrameCount(frame_type);
  size_t num_dofs = positions.size();
  size_t rows = 6 * num_frames;
  size_t cols = num_dofs;
  auto jacobians_array = new double[rows * cols];
  hebiRobotModelGetJacobians(internal_, static_cast<HebiFrameType>(frame_type), positions_array, jacobians_array, HebiMatrixOrderingColumnMajor);
  delete[] positions_array;
  jacobians.resize(num_frames);
  for (size_t i = 0; i < num_frames; ++i) {
    Map<Matrix<double, Dynamic, Dynamic>> tmp(jacobians_array + i * cols * 6, 6, cols);
    jacobians[i] = tmp;
  }
  delete[] jacobians_array;
}
void RobotModel::getJacobianEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const {
  getJEndEffector(positions, jacobian);
}
void RobotModel::getJEndEffector(const Eigen::VectorXd& positions, Eigen::MatrixXd& jacobian) const {
  MatrixXdVector tmp_jacobians;
  getJacobians(FrameType::EndEffector, positions, tmp_jacobians);

  // NOTE: could make this more efficient by writing additional lib function
  // for this, instead of tossing away almost everything from the full one!

  size_t num_dofs = positions.size();
  jacobian.resize(6, num_dofs);
  jacobian = *tmp_jacobians.rbegin();
}

void RobotModel::getMasses(Eigen::VectorXd& masses) const {
  size_t num_masses = getFrameCount(FrameType::CenterOfMass);
  auto masses_array = new double[num_masses];
  hebiRobotModelGetMasses(internal_, masses_array);
  {
    Map<VectorXd> tmp(masses_array, num_masses);
    masses = tmp;
  }
  delete[] masses_array;
}

void RobotModel::getMetadata(std::vector<MetadataBase>& metadata) const {
  const auto num_elems = hebiRobotModelGetNumberOfElements(internal_);
  metadata.resize(num_elems);
  for (auto i = 0; i < num_elems; i++) {
    hebiRobotModelGetElementMetadata(internal_, i, &metadata[i].metadata_);
  }
}

} // namespace robot_model
} // namespace hebi
