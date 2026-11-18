#include "diffdrive_arduino/wheel.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>


void Wheel::setup(const std::string &wheel_name, int counts_per_rev, double wheel_pwm_factor, int min_command_percent)
{
  name = wheel_name;
  if (counts_per_rev <= 0)
  {
    throw std::invalid_argument("Encoder counts per revolution must be positive");
  }
  rads_per_count = (2 * M_PI) / counts_per_rev;

  if (!std::isfinite(wheel_pwm_factor) || wheel_pwm_factor <= 0.001)
  {
    throw std::invalid_argument("Wheel PWM factor must be positive");
  }
  if (min_command_percent < 0 || min_command_percent > 100)
  {
    throw std::invalid_argument("Minimum wheel command percent must be in 0..100");
  }
  this->min_command_percent = min_command_percent;
  this->wheel_pwm_factor = wheel_pwm_factor;
}

double Wheel::calcEncAngle() const
{
  return enc * rads_per_count;
}

int Wheel::roundAndLimit(double val) const
{
  int ival = static_cast<int>(std::round(std::max(-100.0, std::min(100.0, val))));
  return std::max(-100, std::min(100, ival));
}

void Wheel::setEncoderBaseline(int count)
{
  enc = count;
  vel = 0.0;
}

void Wheel::updateEncoder(int count, double elapsed_seconds)
{
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0)
  {
    throw std::invalid_argument("Encoder sample interval must be finite and positive");
  }
  // The protocol uses signed 32-bit counters. Handle rollover in either direction.
  static_assert(sizeof(int) == sizeof(int32_t), "Encoder protocol requires 32-bit integers");
  const uint32_t wrapped_delta = static_cast<uint32_t>(count) - static_cast<uint32_t>(enc);
  const int64_t counts_delta = wrapped_delta <= INT32_MAX ? wrapped_delta :
    static_cast<int64_t>(wrapped_delta) - (int64_t{1} << 32);
  const double delta = counts_delta * rads_per_count;
  pos += delta;
  vel = delta / elapsed_seconds;
  enc = count;
}
