#pragma once

#include <string>

class Range
{
    public:

    std::string name = "";
    double range = 0.0;     // meters

    Range() = default;

    Range(const std::string &range_name);
    
    void setup(const std::string &range_name);

    void setRange(int range_cm);

    double getRange();
};
