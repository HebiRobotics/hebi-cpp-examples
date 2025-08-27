/*
 * Get full pose feedback from a mobile device and visualize online
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
    hebi::charts::GridWindow window;
    auto chart = window.addChart3d();
    window.show();
    auto triad = chart.addTriad(0.075);
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
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

