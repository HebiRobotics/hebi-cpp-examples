
#include "matplotlibcpp.h"

std::vector<double> arrange(double min, double max, double spacing, double origin=0.0, bool inclusive=true){
  std::vector<double> ret;
  for (double i = origin; i < max; i += spacing){
    ret.push_back(i);
  }
  for (double i = origin - spacing; i > min; i -= spacing){
    ret.push_back(i);
  }
  if(inclusive) {
    ret.push_back(max);
    ret.push_back(min);
  }
  return ret;
}

void adjust_limits(double* min, double* max, std::vector<double> data, double scale=2.0){
  for(size_t i = 0; i < data.size(); i++) {
    if (data[i] < *min || data[i] > *max) {
      *min *= scale;
      *max *= scale;
      return;
    }
  }
  bool shrink = true;
  for (size_t i = 0; i < data.size(); i++) {
    if(!(data[i] < *max/scale && data[i] > *min/scale)){
      shrink = false;
    }
  }
  if(shrink){
    *min /= scale;
    *max /= scale;
  }
}

std::vector<double> linspace(double min, double max, int count){
  double spacing = (max-min)/static_cast<double>(count);
  return arrange(min, max, spacing, min);
}

