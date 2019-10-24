#pragma once

#include <cmath>

#include "robot/hexapod_parameters.hpp"

class IHexapodParameterView : public QObject
{
public:
  virtual ~IHexapodParameterView() noexcept = default;

  virtual void resetTransforms() = 0;

  float convertSliderToFloat(int slider, float min, float max)
  {
    float scaled_value = ((float)slider) / 100.f;
    scaled_value *= (max - min);
    scaled_value += min;
    return scaled_value;
  }
  int convertStanceRadiusToSlider(float scaled)
  {
    scaled -= _min_stance_radius;
    scaled /= (_max_stance_radius - _min_stance_radius);
    return (int)std::round(scaled * 100.0f);
  }
  int convertBodyHeightToSlider(float scaled)
  {
    scaled -= _min_body_height;
    scaled /= (_max_body_height - _min_body_height);
    return (int)std::round(scaled * 100.0f);
  }

  virtual const hebi::HexapodParameters* params() { return &_params; }


public slots:
  virtual void stanceRadiusChanged(int new_value)
  {
    _params.stance_radius_ = convertSliderToFloat(new_value, _min_stance_radius, _max_stance_radius);
    resetTransforms();
  }
  virtual void bodyHeightChanged(int new_value)
  {
    _params.default_body_height_ = convertSliderToFloat(new_value, _min_body_height, _max_body_height);
    resetTransforms();
  }
  void paramsUpdated(const hebi::HexapodParameters& params)
  {
    _params = params;
  }

protected:
  hebi::HexapodParameters _params;

  float _min_stance_radius = 0.05f; // m
  float _max_stance_radius = 0.35f; // m
  float _min_body_height = -0.5f; // m
  float _max_body_height = 0.05f; // m
  
};
