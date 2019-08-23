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
// #include "quaternion_f.hpp"

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


  std::vector<double> y;
  Quaternionf mobile_quaternion();
  // std::vector<int64_t> buttons;
  // std::vector<float> sliders(8); // we know we have 8 pins
  // std::vector<std::string> x_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
  // std::vector<double> x_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  std::cout << "\n Visualizing orientation estimate from the mobile device."
            << "\n Move it around to make the feedback interesting..." 
            << std::endl;

  for (size_t i = 0; i < 50; ++i)
  {
    if (group -> getNextFeedback(group_fbk))
    {
      // Obtain feedback for a singular module from the groupFeedback object
      // auto& buttons_data = group_fbk[0].io();
      // put in a check here to see if there is data to get, otherwise stay empty
      // auto& orient_data = group_fbk[0].QuaternionfField();
      auto orient = group_fbk[0].mobile().arOrientation().get();

      Eigen::Quaterniond q;
      q.w() = orient.getW();
      q.x() = orient.getX();
      q.y() = orient.getY();
      q.z() = orient.getZ();

      auto rot_matrix = q.toRotationMatrix();

      Eigen::Matrix4d final_transform;
      final_transform << rot_matrix(0,0), rot_matrix(0,1), rot_matrix(0,2), 0,
                         rot_matrix(1,0), rot_matrix(1,1), rot_matrix(1,2), 0,
                         rot_matrix(2,0), rot_matrix(2,1), rot_matrix(2,2), 0,
                         0, 0, 0, 1;

      // std::cout << 

      std::vector<std::vector<double>> lines_x;
      std::vector<std::vector<double>> lines_y;
      std::vector<std::vector<double>> lines_z;      

      // std::cout << final_transform << std::endl;

      plt::clf();
      plot_3dtriad(final_transform, &lines_x, &lines_y, &lines_z);
      // plt::ylim(-1, 1);
      // plt::xlim(-1, 1);
      // plt::zlim(-1, 1);
      plt::pause(0.01);

      // Now we plot the collected feedback
      // plt::clf();
      // plt::ylim(-3.14, 3.14);
      // plt::xticks(x_ticks, x_labels);
      // plt::xlabel("Axis");
      // plt::ylabel("Angular Velocity (rad/s)");
      // plt::bar(buttons);
      // plt::pause(0.01);
      // plt::bar({1,2,3});
      // plt::show();

      // plt::plot_surface();
      // plt::show();

      // std::vector<std::vector<double>> x, y, z;
      // for (double i = -5; i <= 5;  i += 0.25) {
      //     std::vector<double> x_row, y_row, z_row;
      //     for (double j = -5; j <= 5; j += 0.25) {
      //         x_row.push_back(i);
      //         y_row.push_back(j);
      //         z_row.push_back(::std::sin(::std::hypot(i, j)));
      //     }
      //     x.push_back(x_row);
      //     y.push_back(y_row);
      //     z.push_back(z_row);
      // }

      // plt::plot_surface(x, y, z);
      // plt::show();
    }
  }

  group -> clearFeedbackHandlers();
  return 0;
}

