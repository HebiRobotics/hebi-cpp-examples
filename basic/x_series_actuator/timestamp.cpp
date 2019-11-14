#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "lookup.hpp"
#include "group.hpp"
#include "group_feedback.hpp"

using namespace hebi;

int main() {

  Lookup lookup;
  std::string family_name("Arm");
  std::string module_name("Base");
  auto group = lookup.getGroupFromNames({family_name}, {module_name});

  GroupFeedback fbk(1);
  double duration = 1.0;
  group->getNextFeedback(fbk);
  double start = fbk.getTime();
  double t = 0;

  printf("start = %f\n", start);
  printf("t = %f\n", t);

  while (t < duration) {
    group->getNextFeedback(fbk);
    t = fbk.getTime() - start;
    printf("t = %f\n", t);
  }

  return 0;
}
