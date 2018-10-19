#pragma once

#include "group.hpp"
#include "group_command.hpp"

#include "leg.hpp"
#include "hexapod_parameters.hpp"

#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>

// Leg Numbering / Chassis Coordinate convention
// This should match ROS wheeled vehicle convention
//
//  1 \   / 2         +x
//     \ /            ^
//  3 ----- 4         |
//     / \      +y <--o
//  5 /   \ 6          +z
//
// Distances based on CAD on October 17 2018

using namespace Eigen;

namespace hebi {

struct HexapodErrors {
  bool has_valid_initial_feedback;
  bool m_stop_pressed;
  int first_out_of_range_leg;
};

class Hexapod
{
public:
  enum Mode { Step, Stance };

  static std::unique_ptr<Hexapod> create(const HexapodParameters& params, HexapodErrors& hex_errors);
  static std::unique_ptr<Hexapod> createPartial(const HexapodParameters& params, std::set<int> real_legs, HexapodErrors& hex_errors);
  static std::unique_ptr<Hexapod> createDummy(const HexapodParameters& params);

  static double getFeedbackPeriodMs() { return 5; }

  virtual ~Hexapod() noexcept;

  void startLogging();

  // Updates the hexapod/leg stance variables based on the latest feedback
  void updateStance(const Eigen::Vector3d& translation_velocity, const Eigen::Vector3d& rotation_velocity, double dt);

  // TODO: add velocities/torques
  void setCommand(int leg_index, const VectorXd* angles, const VectorXd* vels, const VectorXd* accels);

  // Actually send the command to the robot.
  void sendCommand();

  bool setGains();

  // toggle the mode a specified number of times
  void updateMode(int num_toggles);

  Eigen::Matrix4d getBodyPoseFromFeet();

  bool needToStep();

  bool isStepping();

  void startStep(double t);

  void updateSteps(double t);

  Mode getMode() { return mode_; }

  Leg* getLeg(int leg_index) { return legs_[leg_index].get(); }

  Eigen::VectorXd getLegFeedback(int leg_index);

  void computeFootForces(double t, Eigen::MatrixXd& foot_forces);

  std::chrono::time_point<std::chrono::steady_clock> getLastFeedbackTime();

  void setLoggingFrequency(double f)
  {
    if (log_group_input_) log_group_input_->setFeedbackFrequencyHz(f);
    if (log_group_modules_) log_group_modules_->setFeedbackFrequencyHz(f);
  }
  bool hasLogGroup() { if (log_group_input_ || log_group_modules_) return true; return false; }

  void clearLegColors();
  void setLegColor(int leg_index, uint8_t r, uint8_t g, uint8_t b);

private:

  std::chrono::time_point<std::chrono::steady_clock> last_fbk;

  // NOTE: remainder are "dummy" legs.
  std::set<int> real_legs_;
  std::set<int> last_step_legs_;

  Hexapod(std::shared_ptr<Group> group, std::shared_ptr<Group> log_group_input, std::shared_ptr<Group> log_group_modules, const HexapodParameters& params, const std::set<int>& real_legs, HexapodErrors& hex_errors);
  std::shared_ptr<Group> group_;
  std::shared_ptr<Group> log_group_input_;
  std::shared_ptr<Group> log_group_modules_;
  hebi::GroupCommand cmd_;
  Eigen::VectorXd positions_;
  std::mutex fbk_lock_;
  std::vector<std::unique_ptr<Leg> > legs_;

  std::chrono::time_point<std::chrono::steady_clock> pose_start_time_;
  double pose_last_time_;

  const int num_legs_ = 6;
  const int num_angles_ = num_legs_ * Leg::getNumJoints();

  // TODO: abstract into "step" class? At least parameter data structure?
  // TODO: LOAD FROM XML!
  HexapodParameters params_;
  static constexpr float weight_ = 9.8f * 21.0f; // mass = 21 kg

  Eigen::Vector3d vel_xyz_;

  Mode mode_;

  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace hebi
