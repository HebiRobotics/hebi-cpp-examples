/*
 * Using full pose feedback from a mobile device, visualize online, 
 * log in the background, and visualize the data from the logs.
 * 
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * HEBI Robotics
 * August 2019
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <iomanip>
 
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "hebi_charts.hpp"
#include "log_file.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**) {
  // Find your module on the network 
  // You can also plot feedback from multiple modules by including multiple modules
  // in your group. Look at example 01c on how to do that.
  Lookup lookup;
  std::string family_name("HEBI");
  std::string module_name("Mobile IO");
  auto group = lookup.getGroupFromNames({family_name}, {module_name});

  // Confirm the module is found before preceding
  if (!group) {
    std::cout << "Group not found!" << std::endl;
    return -1;
  }

  // Set the feedback frequency. 
  // This is by default "100"; setting this to 5 here allows the console output
  // to be more reasonable.
  group->setFeedbackFrequencyHz(5);

  // Retrieve feedback with a blocking all to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run 
  // for about 10 seconds here
  GroupFeedback group_fbk(group->size());

  std::cout << "\n Visualizing 6-DoF Pose estimate from the mobile device."
            << "\n Move it around to make the feedback interesting..." 
            << std::endl;

  if (hebi::charts::lib::isAvailable()) {
    // Start logging (you can also specify log file name as second parameter)
    std::string log_path = group->startLog("./logs");

    if (log_path.empty()) {
      std::cout << "~~ERROR~~\n"
                << "Target directory for log file not found!\n"
                << "HINT: Remember that the path declared in 'group->startLog()' "
                << "is relative to your current working directory...\n";
      return 1;
    }

    hebi::charts::GridWindow orient_window;
    auto orient_chart = orient_window.add3dChart();
    orient_window.show();
    auto triad = orient_chart.addTriad(0.075);
    for (size_t i = 0; i < 50; ++i)
    {
      if (group->getNextFeedback(group_fbk))
      {
        // Obtain feedback for a singular module from the groupFeedback object
        auto orient = group_fbk[0].mobile().arOrientation().get();
        auto pos = group_fbk[0].mobile().arPosition().get();

        // Plot the 6-Dof Pose
        triad.setOrientation(orient.getW(), orient.getX(), orient.getY(), orient.getZ());
        triad.setTranslation(pos.getX(), pos.getY(), pos.getZ());
      }
    }
  
    // Stop logging
    std::shared_ptr<LogFile> log_file = group -> stopLog();

    // Gather the logged Position data
    std::vector<double> x_pos;
    std::vector<double> y_pos;
    std::vector<double> z_pos;
    std::vector<double> times;
    double t0{};
    GroupFeedback fbk(group->size());
    while (log_file -> getNextFeedback(fbk)) 
    {
      x_pos.push_back(fbk[0].mobile().arPosition().get().getX());
      y_pos.push_back(fbk[0].mobile().arPosition().get().getY());
      z_pos.push_back(fbk[0].mobile().arPosition().get().getZ());
      if (t0 == 0)
        t0 = fbk.getTime();
      times.push_back(fbk.getTime() - t0);
    }

    // Plot the logged Position data
    hebi::charts::GridWindow pos_window;
    auto pos_chart = pos_window.addLineChart();
    pos_chart.setTitle("Device Position by Axis (x/y/z)");
    pos_chart.getAxisY().setName("Position (m)");
    auto chart_x = pos_chart.addLine("X Position", times, x_pos);
    auto chart_y = pos_chart.addLine("Y Position", times, y_pos);
    auto chart_z = pos_chart.addLine("Z Position", times, z_pos);
    chart_x.setColor(hebi::charts::Color::Red);
    chart_y.setColor(hebi::charts::Color::Blue);
    chart_z.setColor(hebi::charts::Color::Magenta); // TODO: black
    pos_window.show();

    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

