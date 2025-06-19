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

  if (hebi::charts::framework::isLoaded()) {
    std::vector<double> buttons;
    buttons.resize(8,0); // we know we have 8 pins
    std::vector<double> sliders;
    sliders.resize(8,0); // we know we have 8 pins
    std::vector<std::string> input_labels = {"1", "2", "3", "4", "5", "6", "7", "8"};
    std::vector<double> input_ticks = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    
    std::vector<double> gyro_data;
    std::vector<std::string> gyro_labels = {"X", "Y", "Z"};
    std::vector<double> gyro_ticks = {0.0, 1.0, 2.0};

    hebi::charts::Chart chart_inputs;
    hebi::charts::Chart chart_gyro;
    chart_inputs.setTitle("Mobile I/O Input Feedback");
    chart_gyro.setTitle("Mobile I/O Gyro Feedback");
    chart_inputs.getAxisY().setLimits(-1, 1);
    chart_gyro.getAxisY().setLimits(-3.14, 3.14);
    chart_inputs.getAxisX().setName("Digital Inputs and Analog Inputs");
    chart_gyro.getAxisX().setName("Axis");
    chart_inputs.getAxisY().setName("[-1 to 1]");
    chart_gyro.getAxisY().setName("Angular Velocity (rad/s)");
    // TODO:
    //chart_inputs.getAxisX().setNames(input_labels);
    //chart_gyros.getAxisX().setNames(input_labels);
    //chart_inputs.getAxisX().setTicks(input_ticks);
    //chart_gyros.getAxisX().setTicks(input_ticks);

    auto gyro_chart_data = chart_gyro.addBars("X/Y/Z", gyro_ticks, gyro_data);
    auto button_chart_data = chart_inputs.addBars("Buttons", input_ticks, buttons);
    auto slider_chart_data = chart_inputs.addBars("Sliders", input_ticks, sliders);
    chart_gyro.show();
    chart_inputs.show();
    for (size_t i = 0; i < 50; ++i)
    {
      if (group->getNextFeedback(group_fbk))
      {
        // Obtain feedback for a singular module from the groupFeedback object
        auto& buttons_data = group_fbk[0].io();

        // Digital Feedback (Buttons) 
        // We can safely assume that all buttons return an int value
        // Store as double for interop with the plotting library
        buttons = {static_cast<double>(buttons_data.b().getInt(1)),
                   static_cast<double>(buttons_data.b().getInt(2)),
                   static_cast<double>(buttons_data.b().getInt(3)),
                   static_cast<double>(buttons_data.b().getInt(4)),
                   static_cast<double>(buttons_data.b().getInt(5)),
                   static_cast<double>(buttons_data.b().getInt(6)),
                   static_cast<double>(buttons_data.b().getInt(7)),
                   static_cast<double>(buttons_data.b().getInt(8))};

        // Analog Feedback (Sliders) 
        // We expect float values, but may recieve an int in certain cases.
        // As such, we convert any ints we encounter back to float
        for (size_t i = 0; i < 8; ++i)
        {
          // we check pins i+1 because the pins are numbered 1-8
          if (buttons_data.a().hasFloat(i+1)) {
            sliders[i] = buttons_data.a().getFloat(i+1);
          } else {
            sliders[i] = static_cast<double>(buttons_data.a().getInt(i+1));
          }
        }

        // Gyro Feedback
        auto gyro = group_fbk.getGyro();
        gyro_data = {gyro(0,0), gyro(0,1), gyro(0,2)};

        // Now we plot the collected data
        button_chart_data.setData(input_ticks, buttons);
        slider_chart_data.setData(input_ticks, sliders);
        gyro_chart_data.setData(gyro_ticks, gyro_data);
      }
    }
    hebi::charts::framework::waitUntilWindowsClosed();
  }

  return 0;
}

