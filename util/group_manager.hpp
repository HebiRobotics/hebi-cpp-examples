#pragma once

// HEBI C++ API components
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"

namespace hebi {
namespace experimental {

class GroupManager {

public:
  // Parameters for creating a group manager
  struct Params {
    // The family and names passed to the "lookup" function to find modules
    // Both are required.
    std::vector<std::string> families_;
    std::vector<std::string> names_;
    // How long a command takes effect for on the robot before expiring.
    int command_lifetime_ = 100;
    // Loop rate, in Hz.  This is how fast the arm update loop will nominally
    // run.
    double control_frequency_ = 200.f;

    // A function pointer that returns a double representing the current time in
    // seconds. (Can be overloaded to use, e.g., simulator time)
    //
    // The default value uses the steady clock wall time.
    std::function<double()> get_current_time_s_ = []() {
      using clock = std::chrono::steady_clock;
      static const clock::time_point start_time = clock::now();
      return (std::chrono::duration<double>(clock::now() - start_time)).count();
    }; 
  };

  //////////////////////////////////////////////////////////////////////////////
  // Setup functions
  //////////////////////////////////////////////////////////////////////////////

  // Creates an "Trajectory Follower" object, and puts it into a "weightless" no-goal control
  // mode.
  static std::unique_ptr<GroupManager> create(const Params& params);

  // Loads gains from the given .xml file, and sends them to the module. Returns
  // false if the gains file could not be found, if these is a mismatch in
  // number of modules, or the modules do not acknowledge receipt of the gains.
  bool loadGains(const std::string& gains_file);

  //////////////////////////////////////////////////////////////////////////////
  // Accessors
  //////////////////////////////////////////////////////////////////////////////

  // Returns the number of modules / DoF in the arm
  size_t size() const { return group_->size(); }

  // Returns the internal group. Not necessary for most use cases.
  const Group& group() const { return *group_; }

  // Returns the command last computed by update, or an empty command object
  // if "update" has never successfully run. The returned command can be
  // modified as desired before it is sent to the robot with the send function.
  GroupCommand& pendingCommand() { return command_; }
  const GroupCommand& pendingCommand() const { return command_; }

  // Returns the last feedback obtained by update, or an empty feedback object
  // if "update" has never successfully run.
  const GroupFeedback& lastFeedback() const { return feedback_; }
  const double dT() const { return dt_; }
  const double lastTime() const { return last_time_; }

  // Updates feedback.
  // To retrieve the feedback, call `getLastFeedback()` after this call.
  // You can modify the command object after calling this.
  //
  // Returns 'false' on a connection problem; true on success.
  bool update();

  // Sends the user set command to the group manager.
  bool send();

protected:
  std::function<double()> get_current_time_s_;
  double last_time_;
  double dt_{ std::numeric_limits<double>::quiet_NaN() };
  std::shared_ptr<Group> group_;

  hebi::GroupFeedback feedback_;
  hebi::GroupCommand command_;

  // Private constructor
  GroupManager(
      std::function<double()> get_current_time_s,
      std::shared_ptr<Group> group):
    get_current_time_s_(get_current_time_s),
    last_time_(get_current_time_s()),
    group_(group),
    feedback_(group->size()),
    command_(group->size()) {}
};

std::unique_ptr<GroupManager> GroupManager::create(const GroupManager::Params& params) {

  // Get the group (scope the lookup object so it is destroyed
  // immediately after the lookup operation)
  std::shared_ptr<Group> group;
  {
    Lookup lookup;
    group = lookup.getGroupFromNames(params.families_, params.names_);
  }
  if (!group) {
    std::cout << "Could not create arm! Check that family and names match actuators on the network.\n";
    return nullptr;
  }

  // Set parameters
  if (!group->setCommandLifetimeMs(params.command_lifetime_)) {
    std::cout << "Could not set command lifetime on group; check that it is valid.\n";
    return nullptr;
  }
  if (!group->setFeedbackFrequencyHz(params.control_frequency_)) {
    std::cout << "Could not set feedback frequency on group; check that it is valid.\n";
    return nullptr;
  }

  // Try to get feedback -- if we don't get a packet in the first N times,
  // something is wrong
  int num_attempts = 0;

  // We need feedback, so we can plan trajectories if that gets called before the first "update"
  GroupFeedback feedback(group->size());
  while (!group->getNextFeedback(feedback)) {
    if (num_attempts++ > 10) {
      std::cout << "Could not communicate with robot; check network connection.\n";
      return nullptr;
    }
  } 

  // Note: once ROS moves up to C++14, we can change this to "make_unique".
  return std::unique_ptr<GroupManager>(new GroupManager(params.get_current_time_s_, group));
}
  
bool GroupManager::loadGains(const std::string& gains_file)
{
  hebi::GroupCommand gains_cmd(group_->size());
  if (!gains_cmd.readGains(gains_file))
    return false;

  return group_->sendCommandWithAcknowledgement(gains_cmd);
}

bool GroupManager::update() {
  double t = get_current_time_s_();

  // Time must be monotonically increasing!
  if (t < last_time_)
    return false;

  dt_ = t - last_time_;
  last_time_ = t;

  if (!group_->getNextFeedback(feedback_))
    return false;

  return true;
}

bool GroupManager::send() {
  return group_->sendCommand(command_);
}

} // namespace experimental
} // namespace hebi
