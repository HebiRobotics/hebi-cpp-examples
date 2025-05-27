/**
 * Send position commands and log in the background.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * September 2018
 */

#include <iostream>
#include <chrono>
#include <thread>
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "log_file.hpp"
#include "hebi_charts.hpp"

using namespace hebi;

//static std::atomic_bool did_start{false};
/*
class StartWrapper {
public:
  StartWrapper(int argc, char** argv) {
    std::cout << "Running!\n";
    runner = std::thread([argc, argv](){
    });
    while (!did_start) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Needs to be called on main...and never returns
    hebi::charts::hebiChartsRunApplication(main2, argc, argv);
    std::cout << "Started Thread.\n";
  }
  ~StartWrapper() {
    // TODO: otherwise?
    std::cout << "Joining Thread.  Perhaps this is where we kill 'main2'?\n";
    runner.join();
  }
private:
  static int main2(int argc, char** argv) {
    did_start = true;
    std::this_thread::sleep_for(std::chrono::seconds(40));
    // TODO: wait here?
    return 0;
  }
  std::thread runner;
};
*/
//std::atomic_bool StartWrapper::did_start = false;

static std::atomic_bool did_plot{false};
int main2(int argc, char** argv)
{
  //StartWrapper wrapper(argc, argv);

  // Get group
  Lookup lookup;
  auto group = lookup.getGroupFromNames({"HEBI"}, {"J3"});

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  //// Open-loop controller (position)

  // The command struct has fields for various commands and settings; for the
  // actuator, we will primarily use position, velocity, and effort.
  //
  // Fields that are not filled in will be ignored when sending.
  GroupCommand group_command(group->size());
  // GroupCommand uses Eigen types for data interchange
  Eigen::VectorXd positions(1);
  // Allocate feedback
  GroupFeedback group_feedback(group->size());
  
  // Start logging in the background
  std::string log_path = group->startLog("logs");

  if (log_path.empty()) {
    std::cout << "~~ERROR~~\n"
              << "Target directory for log file not found!\n"
              << "HINT: Remember that the path declared in 'group->startLog()' "
              << "is relative to your current working directory...\n";
    return 1;
  }

  // Parameters for sin/cos function
  double freq_hz = 0.5;               // [Hz]
  double freq = freq_hz * 2.0 * M_PI; // [rad / sec]
  double amp = M_PI / 4.0;            // [rad] (45 degrees)

  double duration = 10;               // [sec]
  auto start = std::chrono::system_clock::now();

  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  while (t.count() < duration) {
    // Even though we don't use the feedback, getting feedback conveniently
    // limits the loop rate to the feedback frequency
    group->getNextFeedback(group_feedback);

    // Update position set point
    t = std::chrono::system_clock::now() - start;
    positions[0] = amp * std::sin(freq * t.count());
    group_command.setPosition(positions);
    group->sendCommand(group_command);
  }

  // Stop logging
  auto log_file = group->stopLog();
  if (!log_file) {
      std::cout << "~~ERROR~~\n"
                << "Log file not found!\n";
      return 1;
  }

  //plot the logged position data
  std::vector<std::vector<double>> pos;
  double t0{};
  std::vector<double> times;
  pos.resize(group->size());
  GroupFeedback fbk(group->size());
  while(log_file->getNextFeedback(fbk)) {
    for(size_t i = 0; i < group->size(); i++){
      pos[i].push_back(fbk.getPosition()[i]);
    }
    if (t0 == 0)
      t0 = fbk.getTime();
    times.push_back(fbk.getTime() - t0);
  }
  auto chart = hebi::charts::Chart::create();
  for(size_t i = 0; i < group->size(); i++){
    // TODO: memory? is dataset freed or tracked by the chart?  Dataset is released, but is this reference counted internally?
    // (usually this would be a member variable of chart, so would be tracked on there...maybe is still is, or we should add
    // it there...)
    auto title =(std::string("module ") + std::to_string(i));
    chart->addLine(title.c_str(), times.data(), pos[i].data(), times.size());
  }
  chart->show();
  did_plot = true;

  std::cout << "robot thread:\n";
  hebi::charts::JavaFxDebugUtil::printThreadInfo("robot");

  hebi::charts::ChartFramework::waitUntilStagesClosed();

  return 0;
}

static std::atomic_bool did_start{false};
int ui_callback(int argc, char** argv) {
  hebi::charts::JavaFxDebugUtil::printThreadInfo("ui start");
  did_start = true;
  auto main_thread = std::thread([argc, argv](){
    // Wait for plotting thread to initialize:
    while (!did_start)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    main2(argc, argv);
  });
  while (!did_plot)
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "UI callback thread:\n";
  hebi::charts::JavaFxDebugUtil::printThreadInfo("ui");
  main_thread.join();
  return 0;
}

int main(int argc, char** argv) {
  // Never returns:
  hebi::charts::JavaFxDebugUtil::printThreadInfo("main call");
  hebi::charts::hebiChartsRunApplication(ui_callback, argc, argv);
}



/*
static std::atomic_bool did_start{false};
int ui_callback(int argc, char** argv) {
  did_start = true;
  return 0;
}

int main(int argc, char** argv) {
  auto main_thread = std::thread([argc, argv](){
    // Wait for plotting thread to initialize:
    while (!did_start)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    main2(argc, argv);
  });
  // Never returns:
  hebi::charts::hebiChartsRunApplication(ui_callback, argc, argv);
}
*/