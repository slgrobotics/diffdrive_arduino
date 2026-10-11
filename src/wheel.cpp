#include "diffdrive_arduino/wheel.h"

#include <cmath>
#include <limits>
#include <stdexcept>

Wheel::Wheel(const std::string & wheel_name, int counts_per_rev)
{
  setup(wheel_name, counts_per_rev);
}

void Wheel::setup(const std::string & wheel_name, int counts_per_rev)
{
  if (counts_per_rev <= 0) { throw std::invalid_argument("Encoder resolution must be positive"); }
  name = wheel_name;
  rads_per_count = 2.0 * std::acos(-1.0) / counts_per_rev;
  enc = 0;
  cmd = pos = vel = 0.0;
}

void Wheel::seed_encoder(int32_t count)
{
  enc = count;
  vel = 0.0;  // Keep position continuous across activation/reconnection.
}

int64_t Wheel::encoder_delta(int32_t count) const
{
  int64_t delta = static_cast<int64_t>(count) - enc;
  // Interpret the shortest displacement across a signed 32-bit counter rollover.
  constexpr int64_t range = int64_t{1} << 32;
  if (delta > std::numeric_limits<int32_t>::max()) { delta -= range; }
  if (delta < std::numeric_limits<int32_t>::min()) { delta += range; }
  return delta;
}

void Wheel::update_encoder(int32_t count, double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0) {
    throw std::invalid_argument("Encoder sampling period must be finite and positive");
  }
  const double displacement = encoder_delta(count) * rads_per_count;
  const double next_position = pos + displacement;
  const double next_velocity = displacement / seconds;
  if (!std::isfinite(next_position) || !std::isfinite(next_velocity)) {
    throw std::invalid_argument("Encoder update would produce nonfinite state");
  }
  pos = next_position;
  vel = next_velocity;
  enc = count;
}

double Wheel::calcEncAngle() const
{
  return enc * rads_per_count;
}
