#pragma once
#include "util/matplotlibcpp.h"

namespace plt = matplotlibcpp;

template<typename T>
std::vector<T> arrange(T min, T max, T spacing, T origin=0.0, bool inclusive=true){
  std::vector<T> ret;
  for (T i = origin; i < max; i += spacing){
    ret.push_back(i);
  }
  for (T i = origin - spacing; i > min; i -= spacing){
    ret.push_back(i);
  }
  if(inclusive) {
    ret.push_back(max);
    ret.emplace(ret.begin(),min);
  }
  return ret;
}

template<typename T>
void adjust_limits(T* min, T* max, std::vector<T> data, T scale=2.0){
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

template<typename F, typename T>
std::vector<T> f_x(std::vector<T> x, F f) {
  std::vector<T> output;
  for (size_t i = 0; i < x.size(); i++) {
    output.push_back(f(x[i]));
  }
  return output;
}

template<typename T>
std::vector<T> linspace(T min, T max, int count){
  T spacing = (max-min)/static_cast<T>(count);
  return arrange(min, max, spacing, min);
}

template<typename T>
void plot_3dtriad(Eigen::Matrix4d transform, std::vector<std::vector<T>>* lines_x, std::vector<std::vector<T>>* lines_y, std::vector<std::vector<T>>* lines_z, bool rgb=true, T len=0.1){
  Eigen::Matrix<double,4,2> line_x, line_y, line_z;
  line_x << 0, len,
  	  0, 0,
  	  0, 0,
  	  1, 1;
  line_y << 0, 0,
  	  0, len,
  	  0, 0,
  	  1, 1;
  line_z << 0, 0,
  	  0, 0,
  	  0, len,
  	  1, 1;
  line_x = transform*line_x;
  line_y = transform*line_y;
  line_z = transform*line_z;
  int index = lines_x->size(); //== lines_y.size() == lines_z.size()
  lines_x->push_back(std::vector<T>({ line_x(0,0),line_x(0,1) }));
  lines_x->push_back(std::vector<T>({ line_y(0,0),line_y(0,1) }));
  lines_x->push_back(std::vector<T>({ line_z(0,0),line_z(0,1) }));
  
  lines_y->push_back(std::vector<T>({ line_x(1,0),line_x(1,1) }));
  lines_y->push_back(std::vector<T>({ line_y(1,0),line_y(1,1) }));
  lines_y->push_back(std::vector<T>({ line_z(1,0),line_z(1,1) }));
  
  lines_z->push_back(std::vector<T>({ line_x(2,0),line_x(2,1) }));
  lines_z->push_back(std::vector<T>({ line_y(2,0),line_y(2,1) }));
  lines_z->push_back(std::vector<T>({ line_z(2,0),line_z(2,1) }));
  
  plt::plot_3dline((*lines_x)[index],(*lines_y)[index],(*lines_z)[index],rgb?"r":"k");
  plt::plot_3dline((*lines_x)[index+1],(*lines_y)[index+1],(*lines_z)[index+1],rgb?"g":"k");
  plt::plot_3dline((*lines_x)[index+2],(*lines_y)[index+2],(*lines_z)[index+2],rgb?"b":"k");
  
}

