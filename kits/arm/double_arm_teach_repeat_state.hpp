/*
 * Contains state structures for keeping from of the state/waypoints of
 * two 6-DoF arms with grippers. Designed to be used with double arm
 * teach repeat example.
 */

#include <set>

#include <Eigen/Eigen>

#include "arm/arm.hpp"

namespace arm = hebi::experimental::arm;

namespace hebi {
namespace examples {

enum class GripperState {
  Open, Close
};

struct Waypoint {
  Eigen::VectorXd positions;
  Eigen::VectorXd vels;
  Eigen::VectorXd accels;
  GripperState gripper_state;
  double time_from_prev;
};
static constexpr double gripperEffort(GripperState gs) {
  return gs == GripperState::Open ? 1 : -5;
}

enum class LeftRight {
  Left, Right
};

// Keeps track of the current waypoints and gripper state for a single arm.
struct ArmState {
  ArmState(int num_modules, hebi::experimental::arm::EndEffectorBase& ee) : num_modules_(num_modules), gripper_(ee),
    prev_gripper_state_(GripperState::Open),
    current_gripper_state_(GripperState::Open)
  {
    Eigen::VectorXd tmp(1);
    tmp << gripperEffort(current_gripper_state_);
    gripper_.update(tmp);
    // Should check the result here...
    gripper_.send();
  }
  int num_modules_{};
  hebi::experimental::arm::EndEffectorBase& gripper_;
  GripperState prev_gripper_state_{GripperState::Open};
  GripperState current_gripper_state_{GripperState::Open};
  std::vector<Waypoint> waypoints_{};
  void toggleGripper() {
    current_gripper_state_ =
      current_gripper_state_ == GripperState::Close ? GripperState::Open : GripperState::Close;
    Eigen::VectorXd tmp(1);
    tmp << gripperEffort(current_gripper_state_);
    // Should check the result here...
    gripper_.update(tmp);
  }
  void reset()
  {
    waypoints_.clear();
    current_gripper_state_ = GripperState::Open;
    prev_gripper_state_ = GripperState::Open;
    Eigen::VectorXd tmp(1);
    tmp << gripperEffort(current_gripper_state_);
    // Should check the result here...
    gripper_.update(tmp);
  }
};

// Keeps track of the current waypoints and gripper state for both arms in the demo.
struct DemoState {
  DemoState(hebi::experimental::arm::Arm& l_arm, hebi::experimental::arm::EndEffectorBase& left_ee,
    hebi::experimental::arm::Arm& r_arm, hebi::experimental::arm::EndEffectorBase& right_ee) :
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

  static void stopArm(experimental::arm::Arm& arm, GripperState gripper_state) {
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

  static std::set<std::string> listSavedWaypoints(const std::map<int, std::string>& waypoint_name_map)
  {
    // TODO: implement
    return {};
  }

  bool loadWaypoints(std::string waypoint_option)
  {
    // TODO: implement
    return true;
  }

  arm::Arm& l_arm_;
  arm::Arm& r_arm_;
  ArmState left_;
  ArmState right_;
};

} // namespace examples
} // namespace hebi


