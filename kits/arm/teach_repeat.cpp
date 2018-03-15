#include "util/input.hpp"
#include "util/grav_comp.hpp"
#include "util/trajectory_time_heuristic.hpp"
#include "arm_container.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "trajectory.hpp"
#include <iostream>
#include <mutex>
#include <vector>
#include <thread>

using namespace hebi::util;
using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector3d = Eigen::Vector3d;

/**
 * Each stored waypoint is defined by its position, velocity, and acceleration;
 * the velocity and acceleration can be "nan" to allow the optimization to
 * freely choose these values at a given waypoint.
 */
struct Waypoint
{
  VectorXd _position;
  VectorXd _velocity;
  VectorXd _acceleration;
};

enum class Mode
{
  Training, Playback 
};

/**
 * The state is shared between the foreground (input) and background (command)
 * threads; this contains the shared objects such as the current list of
 * waypoints and the current application mode.
 */
struct State
{
  std::vector<Waypoint> _waypoints;
  bool _quit {};
  Mode _mode { Mode::Training };
  hebi::ArmContainer& _arm;
  VectorXd _current_position;

  State(hebi::ArmContainer& arm) : _arm(arm) {}
};

/**
 * Protects the "State" object across multiple control threads.
 */
std::mutex state_mutex;

std::shared_ptr<hebi::trajectory::Trajectory> buildTrajectory(State& state)
{
  size_t num_modules = static_cast<size_t>(state._arm.getGroup().size());

  // Reuse the first waypoint as the last one by adding it to the end.
  state._waypoints.push_back(state._waypoints[0]);

  // Build trajectory
  size_t num_waypoints = state._waypoints.size();
  MatrixXd positions(num_modules, num_waypoints);
  MatrixXd velocities(num_modules, num_waypoints);
  MatrixXd accelerations(num_modules, num_waypoints);
  for (size_t i = 0; i < num_waypoints; ++i)
  {
    const auto& waypoint = state._waypoints[i];
    positions.col(i) = waypoint._position;
    velocities.col(i) = waypoint._velocity;
    accelerations.col(i) = waypoint._acceleration;
  }
  VectorXd time_vector = TrajectoryTimeHeuristic::getTimes(positions, velocities, accelerations);
  return hebi::trajectory::Trajectory::createUnconstrainedQp(time_vector, positions, &velocities, &accelerations);
}

/**
 * Background process responsible for getting feedback from the modules,
 * computing the trajectory, and sending commands.
 */
static void commandProc(State* state)
{
  hebi::Group& group = state->_arm.getGroup();
  size_t num_modules = static_cast<size_t>(group.size());
  hebi::GroupFeedback feedback(num_modules);
  hebi::GroupCommand command(num_modules);
  Mode prev_mode = state->_mode;
  auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<hebi::trajectory::Trajectory> trajectory;

  while (true)
  {
    if (!group.getNextFeedback(feedback))
    {
      std::cout << "Did not receive feedback\r\n";
      continue;
    }

    // Acquire lock for duration of this iteration
    std::lock_guard<std::mutex> state_lock(state_mutex);
    if (state->_quit)
      break;

    // Add gravity compensation no matter what
    state->_current_position = feedback.getPosition();
    VectorXd effort = GravityCompensation::getEfforts(
      state->_arm.getRobotModel(),
      state->_arm.getMasses(),
      feedback);
    command.setEffort(effort);

    if (state->_mode == Mode::Playback)
    {
      // First time!
      if (prev_mode != Mode::Playback)
      {
        trajectory = buildTrajectory(*state);

        // Reset time
        start_time = std::chrono::steady_clock::now();
      }

      // Now, actual trajectory playback
      std::chrono::duration<double> time_from_start = std::chrono::steady_clock::now() - start_time;
      double time_in_seconds = time_from_start.count();
      if (time_in_seconds > trajectory->getDuration())
      {
        start_time = std::chrono::steady_clock::now();
        time_in_seconds = 0;
      }
      VectorXd pos(num_modules);
      VectorXd vel(num_modules);
      VectorXd acc(num_modules);
      trajectory->getState(time_in_seconds, &pos, &vel, &acc);
      command.setPosition(pos);
      command.setVelocity(vel);
    }
    if (state->_mode == Mode::Training && prev_mode != Mode::Training)
    {
      // Clear old position commands:
      for (size_t i = 0; i < num_modules; ++i)
        command[i].actuator().position().clear();
    }
    group.sendCommand(command);

    prev_mode = state->_mode;
  }
}

static void addWaypoint(State& state, bool stop)
{
  std::cout << "adding waypoint.\r\n";
  // To ensure smooth playback, make sure the first (and last!) waypoint will be
  // a 'stop' waypoint.
  if (state._waypoints.size() == 0)
    stop = true;

  size_t num_modules = state._current_position.size();
  double vel_accel_val = stop ? 0 : std::numeric_limits<double>::quiet_NaN();

  state._waypoints.push_back( Waypoint{ 
    state._current_position,
    VectorXd::Constant(num_modules, vel_accel_val),
    VectorXd::Constant(num_modules, vel_accel_val)
  });
}

static void clearWaypoints(State& state)
{
  std::cout << "clearing waypoints.\r\n";
  state._waypoints.clear();
}

/**
 * The main function acts as a foreground loop which accepts input from the
 * user.  This executes synchronous actions (add current position as a waypoint,
 * etc) and changes the current mode (e.g., training vs. playback).
 */
int main(int argc, char* argv[])
{
  // Loads the arm configuration -- modify this line to use your own
  // configuration (see kits/arm/arm_container.hpp).
  std::unique_ptr<hebi::ArmContainer> arm = hebi::ArmContainer::create3Dof();
  if (!arm)
  {
    std::cout << "Could not create arm group or object -- ensure all modules are on network.\r\n";
    return -1;
  }
 
  State state(*arm);
 
  std::thread commandThread(commandProc, &state);

  std::cout << "Press 'w' to add waypoint ('s' for stopping at this waypoint), 'c' to clear waypoints, 'p' to playback, and 'q' to quit. \r\n";
  std::cout << "When in playback mode, 't' resumes training, and 'q' quits. \r\n";
  char res = '\0';
  while ((res = Input::getChar()) != 'q')
  {
    // Acquire lock for duration of action step
    std::lock_guard<std::mutex> state_lock(state_mutex);
    std::cout << "\r\n"; // Prettier console output on linux (if echo is enabled)
    if (state._mode == Mode::Training)
    {
      switch(res)
      {
        case 'w':
          addWaypoint(state, false);
          break;
        case 's':
          addWaypoint(state, true);
          break;
        case 'c':
          clearWaypoints(state);
          break;
        case 'p':
          if (state._waypoints.size() >= 2)
            state._mode = Mode::Playback;
          else
            std::cout << "Need at least two waypoints to enter playback mode!\r\n";
          break;
        default:
          continue;
      } 
    }
    if (state._mode == Mode::Playback)
    {
      switch(res)
      {
        case 't':
          state._mode = Mode::Training;
          break;
        default:
          continue;
      } 
    }
  }
  state._quit = true;
  std::cout << "\r\n";

  commandThread.join();
  return 0;
}
