#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <cstdint>
#include <string>

class Wheel
{
public:
  std::string name;
  int32_t enc = 0;
  double cmd = 0;
  double pos = 0;
  double vel = 0;
  double rads_per_count = 0;

  Wheel() = default;
  Wheel(const std::string & wheel_name, int counts_per_rev);
  void setup(const std::string & wheel_name, int counts_per_rev);
  void seed_encoder(int32_t count);
  int64_t encoder_delta(int32_t count) const;
  void update_encoder(int32_t count, double seconds);
  double calcEncAngle() const;
};

#endif
