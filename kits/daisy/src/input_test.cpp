#include "input/input_manager_mobile_io.hpp"
#include <unistd.h>
#include <iostream>

using namespace hebi::input;

int main()
{
  InputManagerMobileIO input;
  if (!input.isConnected())
    return -1;
  Eigen::Vector3f tv, rv;
  while (true)
  {
    if (input.update())
    {
      input.printState();
      if (input.getQuitButtonPushed())
        break;
    }
    usleep(100000);
  }

  return 0;
}
