#ifndef QUADRUPED_STEP_MAPPER__STEP_LENGTH_MAPPER_HPP_
#define QUADRUPED_STEP_MAPPER__STEP_LENGTH_MAPPER_HPP_

#include <cstdint>

namespace quadruped_step_mapper
{

struct StepMappingParameters
{
  double step_frequency_hz{2.5};
  double stance_half_width_m{0.18};
  double max_step_length_m{0.10};
  double max_step_yaw_rad{0.35};
  double max_linear_speed_mps{0.40};
  double max_angular_speed_rps{1.20};
  double min_effective_linear_speed_mps{0.01};
  double min_effective_angular_speed_rps{0.05};
  bool scale_to_limits{true};
};

struct StepCommand
{
  double left_step_length_m{0.0};
  double right_step_length_m{0.0};
  double forward_step_length_m{0.0};
  double step_yaw_rad{0.0};
};

class StepLengthMapper
{
public:
  explicit StepLengthMapper(const StepMappingParameters & parameters);

  StepCommand compute(double vx, double wz) const;

  static int quantizeSignedLevel(
    double step_length_m,
    double max_step_length_m,
    int max_step_level);
  static double normalizeSignedRatio(
    double step_length_m,
    double max_step_length_m);

private:
  static double clampMagnitude(double value, double max_abs_value);

  StepMappingParameters parameters_;
};

}  // namespace quadruped_step_mapper

#endif  // QUADRUPED_STEP_MAPPER__STEP_LENGTH_MAPPER_HPP_
