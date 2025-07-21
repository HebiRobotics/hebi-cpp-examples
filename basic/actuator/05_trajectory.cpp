/**
 * Generate a trajectory and execute it.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * September 2018
 */

#include "trajectory.hpp"
#include "Eigen/Eigen"

#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "command.hpp"
#include <math.h>
#include <chrono>
#include "hebi_charts.hpp"

template<typename T>
std::vector<T> arrange(T min, T max, T spacing, T origin=0.0, bool inclusive=true){
  std::vector<T> ret;
  for (T i = origin; i < max; i += spacing){
    ret.push_back(i);
  }
  for (T i = origin - spacing; i > min; i -= spacing){
    ret.push_back(i);
  }
  if(inclusive) {
    ret.push_back(max);
    ret.emplace(ret.begin(),min);
  }
  return ret;
}

template<typename F, typename T>
std::vector<T> f_x(std::vector<T> x, F f) {
  std::vector<T> output;
  for (size_t i = 0; i < x.size(); i++) {
    output.push_back(f(x[i]));
  }
  return output;
}

template<typename T>
std::vector<T> linspace(T min, T max, int count){
  T spacing = (max-min)/static_cast<T>(count);
  return arrange(min, max, spacing, min);
}

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**)
{
  constexpr double PI = 3.14159265358979323846;
  // Get group
  hebi::Lookup lookup;
  auto group = lookup.getGroupFromNames({"Test Family"}, {"Test Actuator" });

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  // Get the current feedback from the module; this allows us to start at the
  // current position
  int num_joints = group->size();
  hebi::GroupFeedback fbk(num_joints);
  
  if (!group->getNextFeedback(fbk)) {
    printf("Error getting feedback.\n");
    return -1;
  }

  // Create a smooth trajectory starting at the current position, moving to half
  // a rotation away, and coming back to the starting position.
  // Each column is a separate waypoint; each row is a different joint.
  // You can create position, velocity, and acceleration waypoints; the velocity
  // and acceleration waypoints default to zero at the endpoints and
  // unconstrained in the interior points.
  Eigen::MatrixXd positions(num_joints,3);
  auto offset = Eigen::VectorXd::Constant(num_joints, PI);
  auto current_pos = fbk.getPosition();

  positions.col(0) = current_pos;
  positions.col(1) = current_pos + offset;
  positions.col(2) = current_pos;

  // The times to reach each waypoint (in seconds)
  Eigen::VectorXd time(3);
  time << 0, 3, 6;

  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions);

  // Start logging in the background
  std::string log_path = group->startLog("./logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  // Follow the trajectory
  hebi::GroupCommand cmd(num_joints);
  double duration = trajectory->getDuration();
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);

  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);

  while (t.count() < duration) {
    // "getNextFeedback" serves to rate limit the loop without calling sleep
    group->getNextFeedback(fbk);
    t = std::chrono::system_clock::now() - start;

    // Pass "nullptr" in to ignore a term.  Position and time are always returned.
    trajectory->getState(t.count(), &pos_cmd, &vel_cmd, nullptr);
    cmd.setPosition(pos_cmd);
    cmd.setVelocity(vel_cmd);
    group->sendCommand(cmd);
  }
  // Stop logging
  auto log_file = group->stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  //plot graph of trajectories
  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::Chart chart;
    auto x = linspace(0.0,time[2],100.0);
    for (size_t i = 0; i < num_joints; i++) {
      //these are calls to the function f_x which takes a vector of doubles, x, and a lambda, f, and returns a vector f(x)
      std::vector<double> p = f_x(x,[&, i](double t) {Eigen::VectorXd pos(num_joints); trajectory->getState(t,&pos,nullptr,nullptr); return pos[i]; });
      std::vector<double> v = f_x(x,[&, i](double t) {Eigen::VectorXd vel(num_joints); trajectory->getState(t,nullptr,&vel,nullptr); return vel[i]; });
      auto line = chart.addLine("position", x, p);
      line.setColor(hebi::charts::Color::Blue);
      line.setLineStyle(hebi::charts::LineStyle::Solid);
      auto line2 = chart.addLine("velocity", x, v);
      line2.setColor(hebi::charts::Color::Red);
      line2.setLineStyle(hebi::charts::LineStyle::Dashed);
      chart.show();
    }

    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}
