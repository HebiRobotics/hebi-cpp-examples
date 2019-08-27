/*
 * Get orientation feedback from a mobile device and visualize online
 *
 * This example uses blocking. For a non-blocking example, please check
 * example 02f_feedback_background_mobile_io
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
#include "plot_functions.h"

namespace plt = matplotlibcpp;

using namespace hebi;

int main() {
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
  group -> setFeedbackFrequencyHz(5);

  // Retrieve feedback with a blocking all to "getNextFeedback". This
  // constrains the loop to run at the feedback frequency above; we run 
  // for about 10 seconds here
  GroupFeedback group_fbk(group->size());

  std::cout << "\n Visualizing orientation estimate from the mobile device."
            << "\n Move it around to make the feedback interesting..." 
            << std::endl;

  for (size_t i = 0; i < 50; ++i)
  {
    if (group -> getNextFeedback(group_fbk))
    {
      // Obtain feedback for a singular module from the groupFeedback object
      auto orient = group_fbk[0].mobile().arOrientation().get();

      // Derive a rotation matrix from the orientation quaternion
      Eigen::Quaterniond q;
      q.w() = orient.getW();
      q.x() = orient.getX();
      q.y() = orient.getY();
      q.z() = orient.getZ();
      auto rot_matrix = q.toRotationMatrix();

      // Construct a complete 4x4 transform
      Eigen::Matrix4d final_transform;
      final_transform << rot_matrix(0,0), rot_matrix(0,1), rot_matrix(0,2), 0,
                         rot_matrix(1,0), rot_matrix(1,1), rot_matrix(1,2), 0,
                         rot_matrix(2,0), rot_matrix(2,1), rot_matrix(2,2), 0,
                         0, 0, 0, 1;

      // Initialize vectors to pass the plot_3dtriad graphing function
      std::vector<std::vector<double>> lines_x;
      std::vector<std::vector<double>> lines_y;
      std::vector<std::vector<double>> lines_z; 

      // Plot the 3D orientation     
      plt::clf();
      plot_3dtriad(final_transform, &lines_x, &lines_y, &lines_z);
      plt::pause(0.01);
    }
  }

  group -> clearFeedbackHandlers();
  return 0;
}

