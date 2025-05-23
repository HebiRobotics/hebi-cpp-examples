/*
 * Contains state structures for keeping from of the state/waypoints of
 * two 6-DoF arms with grippers. Designed to be used with double arm
 * teach repeat example.
 */

#include <fstream>
#include <set>
#include <string>

#include <Eigen/Eigen>

#include "arm/arm.hpp"

namespace hebi {
namespace examples {

struct Waypoint {
  Eigen::VectorXd positions;
  Eigen::VectorXd vels;
  Eigen::VectorXd accels;
  arm::Gripper::State gripper_state;
  double time_from_prev;
};

constexpr double gripperEffort(arm::Gripper::State gs) {
  return gs == arm::Gripper::State::Open ? 1 : -5;
}

// Estimation for reversing the gripperEffort function
static constexpr arm::Gripper::State gripperState(float effort) {
  // Choose whatever this is closest to:
  return (std::abs(gripperEffort(arm::Gripper::State::Open) - effort) < std::abs(gripperEffort(arm::Gripper::State::Close) - effort)) ? arm::Gripper::State::Open : arm::Gripper::State::Close;
}

enum class LeftRight {
  Left, Right
};

// Keeps track of the current waypoints and gripper state for a single arm.
struct ArmState {
  ArmState(int num_modules, arm::Gripper& gripper) : num_modules_(num_modules), gripper_(gripper),
    prev_gripper_state_(arm::Gripper::State::Open),
    current_gripper_state_(arm::Gripper::State::Open)
  {
    gripper_.setState(current_gripper_state_);
  }
  int num_modules_{};
  arm::Gripper& gripper_;
  arm::Gripper::State prev_gripper_state_{arm::Gripper::State::Open};
  arm::Gripper::State current_gripper_state_{arm::Gripper::State::Open};
  std::vector<Waypoint> waypoints_{};
  void toggleGripper() {
    current_gripper_state_ =
      current_gripper_state_ == arm::Gripper::State::Close ? arm::Gripper::State::Open : arm::Gripper::State::Close;
    gripper_.setState(current_gripper_state_);
  }
  void reset()
  {
    waypoints_.clear();
    current_gripper_state_ = arm::Gripper::State::Open;
    prev_gripper_state_ = arm::Gripper::State::Open;
    gripper_.setState(current_gripper_state_);
  }
};

// Keeps track of the current waypoints and gripper state for both arms in the demo.
struct DemoState {
  DemoState(arm::Arm& l_arm, arm::Gripper& left_ee,
    arm::Arm& r_arm, arm::Gripper& right_ee) :
    l_arm_(l_arm), r_arm_(r_arm),
    left_(l_arm.robotModel().getDoFCount(), left_ee), right_(r_arm.robotModel().getDoFCount(), right_ee)
  {
    
  }
  // Updates current "goal state" with a new waypoint
  bool addWaypoint(float speed, bool stop_waypoint)
  {
    std::cout << "Adding a Waypoint.\n";
    auto l_pos = l_arm_.lastFeedback().getPosition();
    auto r_pos = r_arm_.lastFeedback().getPosition();
    if (stop_waypoint)
    {
      // Stop waypoint with gripper change: wait here for gripper change
      VectorXd l_zeros = VectorXd::Constant(left_.num_modules_, 0);
      VectorXd r_zeros = VectorXd::Constant(right_.num_modules_, 0);
      left_.waypoints_.push_back(Waypoint {l_pos, l_zeros, l_zeros,
                                 left_.current_gripper_state_, speed + 2.5f});
      right_.waypoints_.push_back(Waypoint {r_pos, r_zeros, r_zeros,
                                  right_.current_gripper_state_, speed + 2.5f});

      // If the gripper state changed for either arm here, hang out for bit so the gripper can change state
      if (left_.prev_gripper_state_ != left_.current_gripper_state_ ||
          right_.prev_gripper_state_ != right_.current_gripper_state_)
      {
        left_.waypoints_.push_back(Waypoint {l_pos, l_zeros, l_zeros,
                                   left_.current_gripper_state_, 1.4f});
        right_.waypoints_.push_back(Waypoint {r_pos, r_zeros, r_zeros,
                                    right_.current_gripper_state_, 1.4f});
      }
    }
    else
    {
      VectorXd l_nans = VectorXd::Constant(left_.num_modules_, std::numeric_limits<double>::quiet_NaN());
      VectorXd r_nans = VectorXd::Constant(right_.num_modules_, std::numeric_limits<double>::quiet_NaN());
      left_.waypoints_.push_back(Waypoint {l_pos, l_nans, l_nans, 
                                 left_.current_gripper_state_, speed + 2.5f});
      right_.waypoints_.push_back(Waypoint {r_pos, r_nans, r_nans,
                                  right_.current_gripper_state_, speed + 2.5f});
    }
    // "prev" state is last waypoint state, so this now gets reset:
    left_.prev_gripper_state_ = left_.current_gripper_state_;
    right_.prev_gripper_state_ = right_.current_gripper_state_;
    return true;
  }

  bool toggleGripper(LeftRight left_right)
  {
    if (left_right == LeftRight::Left)
      left_.toggleGripper();
    else if (left_right == LeftRight::Right)
      right_.toggleGripper();
    // (any reason to return anything besides true? failed send?)
    return true;
  }

  bool playWaypoints(bool repeat)
  {
    int wp_count = left_.waypoints_.size();
    // Should never happen!
    if (wp_count != right_.waypoints_.size())
    {
      std::cout << "Internal error! Fix bug in code.\n";
      return false;
    }
    int extra_wps = 0;
    // If the last gripper state doesn't match the first, and we are repeating, we toggle on
    // the way back to the first:
    if (repeat &&
         (left_.waypoints_.begin()->gripper_state != left_.waypoints_.rbegin()->gripper_state ||
          right_.waypoints_.begin()->gripper_state != right_.waypoints_.rbegin()->gripper_state))
    {
      extra_wps = 1;
    }

    Eigen::VectorXd times(wp_count + extra_wps);
    auto makeGoal = [&times, wp_count, extra_wps](ArmState& state){
      // Set up required variables
      Eigen::MatrixXd target_pos(state.num_modules_, wp_count + extra_wps);
      Eigen::MatrixXd target_vels(state.num_modules_, wp_count + extra_wps);
      Eigen::MatrixXd target_accels(state.num_modules_, wp_count + extra_wps);
      Eigen::MatrixXd aux(1, wp_count + extra_wps);

      // Fill up matrices appropriately
      auto t = 0;
      for (int i = 0; i < wp_count; i++)
      {
        const auto& wp = state.waypoints_[i]; 
        t += wp.time_from_prev;
        times[i] = t;
        target_pos.col(i) << wp.positions;
        target_vels.col(i) << wp.vels;
        target_accels.col(i) << wp.accels;
        aux(0, i) = gripperEffort(wp.gripper_state);
      }

      if (extra_wps != 0)
      {
        times[wp_count] = t + 0.5;
        const auto& wp = *state.waypoints_.rbegin(); 
        target_pos.col(wp_count) << wp.positions;
        target_vels.col(wp_count) << wp.vels;
        target_accels.col(wp_count) << wp.accels;
        aux(0, wp_count) = gripperEffort(state.waypoints_[0].gripper_state);
      }
      return arm::Goal::createFromWaypointsWithAux(times, target_pos, target_vels, target_accels, aux);
    };

    l_arm_.setGoal(makeGoal(left_));
    r_arm_.setGoal(makeGoal(right_));
    // (any reason to return anything besides true? failed send?)
    return true;
  } 

  static void stopArm(arm::Arm& arm, arm::Gripper::State gripper_state) {
    auto current_pos = arm.lastFeedback().getPosition();
    auto current_vel = arm.lastFeedback().getVelocity();

    // Set up required variables
    Eigen::VectorXd times(1);
    times[0] = 1.5; // time to stop
    Eigen::MatrixXd target_pos(arm.robotModel().getDoFCount(), 1);
    target_pos.col(0) = current_pos;
    Eigen::MatrixXd target_vels(arm.robotModel().getDoFCount(), 1);
    target_vels.col(0).setConstant(0);
    Eigen::MatrixXd target_accels(arm.robotModel().getDoFCount(), 1);
    target_accels.col(0).setConstant(0);
    Eigen::MatrixXd aux(1, 1);
    aux(0, 0) = gripperEffort(gripper_state);
    arm.setGoal(arm::Goal::createFromWaypointsWithAux(times, target_pos, target_vels, target_accels, aux));
  }

  void reset()
  {
    left_.reset();
    right_.reset();
  }

  int numWaypoints() const {
    return left_.waypoints_.size();
  }

  bool saveWaypoints(std::string waypoint_option)
  {
    int wp_count = left_.waypoints_.size();
    // Should never happen!
    if (wp_count != right_.waypoints_.size())
    {
      std::cout << "Internal error! Fix bug in code.\n";
      return false;
    }
    auto left_filename = waypoint_option + "_left.wtf";
    auto right_filename = waypoint_option + "_right.wtf";
    std::ofstream left_out, right_out;
    left_out.open(left_filename);
    right_out.open(right_filename);

    if (!left_out.is_open() || !right_out.is_open())
      return false;

    // LOCALIZATION MODE! AFFECTS COMMA FOR DECIMAL POINT!
    auto writeWP = [](Waypoint& wp, std::ofstream& file) {
      file << wp.time_from_prev << ", ";
      int n = wp.positions.size();
      for (int i = 0; i < n; ++i)
        file << wp.positions[i] << ", ";
      for (int i = 0; i < n; ++i)
        file << wp.vels[i] << ", ";
      for (int i = 0; i < n; ++i)
        file << wp.accels[i] << ", ";
      file << gripperEffort(wp.gripper_state) << "\n";
    };

    // num modules, num aux
    left_out << left_.num_modules_ << ", 1\n";
    right_out << right_.num_modules_ << ", 1\n";
    for (int i = 0; i < wp_count; ++i)
    {
      writeWP(left_.waypoints_[i], left_out);
      writeWP(right_.waypoints_[i], right_out);
    }

    return true;
  }

  inline static bool exists(const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
      fclose(file);
      return true;
    } else {
      return false;
    }   
  }

  static std::set<std::string> listSavedWaypoints(const std::map<int, std::string>& waypoint_name_map)
  {
    std::set<std::string> founds;
    for (auto& kvp : waypoint_name_map)
    {
      if (exists(kvp.second + "_left.wtf") && exists(kvp.second + "_right.wtf"))
        founds.insert(kvp.second);
    }

    return founds;
  }

  static std::vector<std::string> split(const std::string& s, char seperator)
  {
    std::vector<std::string> output;
    std::string::size_type prev_pos = 0, pos = 0;
    while((pos = s.find(seperator, pos)) != std::string::npos)
    {
        std::string substring( s.substr(prev_pos, pos-prev_pos) );
        output.push_back(substring);
        prev_pos = ++pos;
    }
    output.push_back(s.substr(prev_pos, pos-prev_pos));
    return output;
  }

  enum class LoadResult
  {
    Success, // New waypoints loaded successfully!
    NoFile, // Couldn't access file; existing waypoints unchanged
    Failure // Resets waypoints in this case!
  };

  LoadResult loadWaypoints(std::string waypoint_option)
  {
    auto left_filename = waypoint_option + "_left.wtf";
    auto right_filename = waypoint_option + "_right.wtf";
    std::ifstream left_in, right_in;
    left_in.open(left_filename);
    right_in.open(right_filename);

    if (!left_in.is_open() || !left_in.good() || !right_in.is_open() || !right_in.good())
      return LoadResult::NoFile;

    auto addWP = [](ArmState& state, std::string& line) {
      // Empty line...
      if (line == "")
        return true;
      int dofs = state.num_modules_;
      auto splits = split(line, ',');
      if (splits.size() != 1 + dofs * 3 + 1) // time + (pos, vel, accel) * num dof + gripper
        return false;
      // Note: could have better error checking here
      Waypoint wp;
      wp.time_from_prev = std::stod(splits[0]);
      wp.positions.resize(dofs);
      wp.vels.resize(dofs);
      wp.accels.resize(dofs);
      for (int i = 0; i < dofs; ++i)
        wp.positions[i] = std::stod(splits[1 + i]);
      for (int i = 0; i < dofs; ++i)
        wp.vels[i] = std::stod(splits[1 + dofs + i]);
      for (int i = 0; i < dofs; ++i)
        wp.accels[i] = std::stod(splits[1 + 2 * dofs + i]);
      auto raw_gripper = std::stod(splits[3 * dofs + 1]);
      wp.gripper_state = gripperState(raw_gripper);
      
      state.waypoints_.push_back(wp);
      
      return true;
    };

    left_.reset();
    right_.reset();

    // Metadata:
    std::string line;
    if (!std::getline(left_in, line))
      return LoadResult::Failure;
    {
      auto left_meta = split(line, ',');
      if (left_meta.size() != 2)
        return LoadResult::Failure;
      // Check for single end effector!
      if (std::stod(left_meta[1]) != 1)
        return LoadResult::Failure;
      // Check that it is the correct number of DOFs
      if (std::stod(left_meta[0]) != left_.num_modules_)
        return LoadResult::Failure;
    }
    if (!std::getline(right_in, line))
      return LoadResult::Failure;
    {
      auto right_meta = split(line, ',');
      if (right_meta.size() != 2)
        return LoadResult::Failure;
      // Check for single end effector!
      if (std::stod(right_meta[1]) != 1)
        return LoadResult::Failure;
      // Check that it is the correct number of DOFs
      if (std::stod(right_meta[0]) != right_.num_modules_)
        return LoadResult::Failure;
    }
    while (std::getline(left_in, line))
    {
      bool success = addWP(left_, line);
      if (!success)
      {
        left_.reset();
        return LoadResult::Failure;
      } 
    }
    while (std::getline(right_in, line))
    {
      bool success = addWP(right_, line);
      if (!success)
      {
        left_.reset();
        right_.reset();
        return LoadResult::Failure;
      }
    }

    // Check for consistency:
    if (left_.waypoints_.size() != right_.waypoints_.size())
    {
      left_.reset();
      right_.reset();
      return LoadResult::Failure;
    }

    return LoadResult::Success;
  }

  arm::Arm& l_arm_;
  arm::Arm& r_arm_;
  ArmState left_;
  ArmState right_;
};

} // namespace examples
} // namespace hebi


