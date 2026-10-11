#pragma once

#include <string>

class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev);
    
    void setup(const std::string &wheel_name, int counts_per_rev);

    double calcEncAngle();

    // Rebase raw counters without changing the exposed joint position.
    void setEncoderBaseline(int count);
    void updateEncoder(int count, double elapsed_seconds);
};
