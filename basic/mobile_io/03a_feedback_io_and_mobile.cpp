/*
 * Simultaneously read analog, digital inputs, and gyro feedback and 
 * visualize online
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
 
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "hebi_charts.hpp"

using namespace hebi;

int run(int, char**);
int main(int argc, char** argv) {
  hebi::charts::runApplication(run, argc, argv);
}
int run(int, char**) {
  // Find your mobile device on the network 
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

  std::cout << "\n Drag the Sliders, press some buttons, and move the device..." 
            << std::endl;

  if (hebi::charts::lib::isAvailable()) {
    hebi::charts::Window window;
    window.setGridSize(1, 2);
    auto chart_inputs = window.addChart(0, 0, 1, 1);
    auto chart_gyro = window.addChart(0, 1, 1, 1);
    chart_inputs.setTitle("Mobile I/O Input Feedback");
    chart_gyro.setTitle("Mobile I/O Gyro Feedback");
    chart_inputs.getAxisY().setLimits(-1, 1);
    chart_gyro.getAxisY().setLimits(-3.14, 3.14);
    chart_inputs.getAxisX().setName("timestep");
    chart_gyro.getAxisX().setName("timestep");
    chart_inputs.getAxisY().setName("[-1 to 1]");
    chart_gyro.getAxisY().setName("rad/s");

    std::array<hebi::charts::Dataset, 8> button_chart_data = {
      chart_inputs.addLine("Button 1", {}, {}),
      chart_inputs.addLine("Button 2", {}, {}),
      chart_inputs.addLine("Button 3", {}, {}),
      chart_inputs.addLine("Button 4", {}, {}),
      chart_inputs.addLine("Button 5", {}, {}),
      chart_inputs.addLine("Button 6", {}, {}),
      chart_inputs.addLine("Button 7", {}, {}),
      chart_inputs.addLine("Button 8", {}, {})
    };
    std::array<hebi::charts::Dataset, 8> slider_chart_data = {
      chart_inputs.addLine("Slider 1", {}, {}),
      chart_inputs.addLine("Slider 2", {}, {}),
      chart_inputs.addLine("Slider 3", {}, {}),
      chart_inputs.addLine("Slider 4", {}, {}),
      chart_inputs.addLine("Slider 5", {}, {}),
      chart_inputs.addLine("Slider 6", {}, {}),
      chart_inputs.addLine("Slider 7", {}, {}),
      chart_inputs.addLine("Slider 8", {}, {})
    };

    auto gyro_x_data = chart_gyro.addLine("X", {}, {});
    auto gyro_y_data = chart_gyro.addLine("Y", {}, {});
    auto gyro_z_data = chart_gyro.addLine("Z", {}, {});
    window.show();
    for (size_t i = 0; i < 50; ++i)
    {
      if (group->getNextFeedback(group_fbk))
      {
        // Obtain feedback for a singular module from the groupFeedback object
        auto& io_data = group_fbk[0].io();

        // Digital Feedback (Buttons) 
        // We can safely assume that all buttons return an int value
        // Store as double for interop with the plotting library
        // Note -- here and below we check pins i+1 because the pins are numbered 1-8, not 0-7
        for (size_t j = 0; j < 8; ++j)
          button_chart_data[j].addPoint(i, static_cast<double>(io_data.b().getInt(j + 1)));

        // Analog Feedback (Sliders) 
        // We expect float values, but may recieve an int in certain cases.
        // As such, we convert any ints we encounter back to float
        for (size_t j = 0; j < 8; ++j)
        {
          double slider_value = 0;
          if (io_data.a().hasFloat(j + 1)) {
            slider_value = io_data.a().getFloat(j + 1);
          } else {
            slider_value = static_cast<double>(io_data.a().getInt(j + 1));
          }
          slider_chart_data[j].addPoint(i, slider_value);
        }

        // Gyro Feedback
        auto gyro = group_fbk.getGyro();
        gyro_x_data.addPoint(i, gyro(0, 0));
        gyro_y_data.addPoint(i, gyro(0, 1));
        gyro_z_data.addPoint(i, gyro(0, 2));
      }
    }
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

