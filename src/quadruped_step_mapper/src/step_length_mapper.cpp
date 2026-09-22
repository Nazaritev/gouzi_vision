#include "quadruped_step_mapper/step_length_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quadruped_step_mapper
{

StepLengthMapper::StepLengthMapper(const StepMappingParameters & parameters)
: parameters_(parameters)
{
  if (parameters_.step_frequency_hz <= 0.0) {
    throw std::runtime_error("step_frequency_hz must be > 0.");
  }
  if (parameters_.stance_half_width_m < 0.0) {
    throw std::runtime_error("stance_half_width_m must be >= 0.");
  }
  if (parameters_.max_step_length_m < 0.0) {
    throw std::runtime_error("max_step_length_m must be >= 0.");
  }
  if (parameters_.max_step_yaw_rad < 0.0) {
    throw std::runtime_error("max_step_yaw_rad must be >= 0.");
  }
}

StepCommand StepLengthMapper::compute(double vx, double wz) const
{
  StepCommand command;

  const double step_period = 1.0 / parameters_.step_frequency_hz;

  vx = clampMagnitude(vx, parameters_.max_linear_speed_mps);
  wz = clampMagnitude(wz, parameters_.max_angular_speed_rps);

  if (std::abs(vx) < parameters_.min_effective_linear_speed_mps) {
    vx = 0.0;
  }
  if (std::abs(wz) < parameters_.min_effective_angular_speed_rps) {
    wz = 0.0;
  }

  double left_step = (vx - parameters_.stance_half_width_m * wz) * step_period;
  double right_step = (vx + parameters_.stance_half_width_m * wz) * step_period;
  double step_yaw = wz * step_period;

  if (parameters_.scale_to_limits) {
    double scale = 1.0;

    if (parameters_.max_step_length_m > 0.0) {
      const double required_step_length = std::max(std::abs(left_step), std::abs(right_step));
      if (required_step_length > parameters_.max_step_length_m) {
        scale = std::min(scale, parameters_.max_step_length_m / required_step_length);
      }
    }

    if (parameters_.max_step_yaw_rad > 0.0) {
      const double required_step_yaw = std::abs(step_yaw);
      if (required_step_yaw > parameters_.max_step_yaw_rad) {
        scale = std::min(scale, parameters_.max_step_yaw_rad / required_step_yaw);
      }
    }

    left_step *= scale;
    right_step *= scale;
    step_yaw *= scale;
  }

  command.left_step_length_m = left_step;
  command.right_step_length_m = right_step;
  command.forward_step_length_m = 0.5 * (left_step + right_step);
  command.step_yaw_rad = step_yaw;
  return command;
}

int StepLengthMapper::quantizeSignedLevel(
  double step_length_m,
  double max_step_length_m,
  int max_step_level)
{
  if (max_step_length_m <= 0.0 || max_step_level <= 0) {
    return 0;
  }

  const double normalized = clampMagnitude(step_length_m / max_step_length_m, 1.0);
  const double scaled = normalized * static_cast<double>(max_step_level);
  return static_cast<int>(std::lround(scaled));
}

double StepLengthMapper::normalizeSignedRatio(
  double step_length_m,
  double max_step_length_m)
{
  if (max_step_length_m <= 0.0) {
    return 0.0;
  }

  return clampMagnitude(step_length_m / max_step_length_m, 1.0);
}

double StepLengthMapper::clampMagnitude(double value, double max_abs_value)
{
  if (max_abs_value <= 0.0) {
    return value;
  }

  return std::max(-max_abs_value, std::min(value, max_abs_value));
}

}  // namespace quadruped_step_mapper
