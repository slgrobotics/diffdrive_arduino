#pragma once

#include <string>

class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;  // speed setpoints, any units, will be scaled by wheel_pwm_factor to Arduino's -100..100 range
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;
    double wheel_pwm_factor = 1.0;  // scaling factor to convert velocity command to Arduino's -100..100 range

    Wheel() = default;
    
    void setup(const std::string &wheel_name, int counts_per_rev, double wheel_pwm_factor = 1.0);

    double calcEncAngle() const;

    int roundAndLimit(double val) const;

    inline int getRawCmd() const
    {
      // Arduino expects wheel speed setpoints in the range -100..100 (percent of max speed)
      // Use wheels_pwm_factor parameter to bring interface speed input into this range
      return roundAndLimit(cmd * wheel_pwm_factor);
    }

    // Rebase raw counters without changing the exposed joint position.
    void setEncoderBaseline(int count);
    void updateEncoder(int count, double elapsed_seconds);
};
