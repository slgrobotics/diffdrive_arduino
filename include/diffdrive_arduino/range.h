#pragma once

#include <string>
#include <limits>

class Range
{
    public:

    std::string name = "";
    double range = 0.0;     // meters

    Range() = default;

    Range(const std::string &range_name);
    
    void setup(
      const std::string &range_name,
      double no_return_range = std::numeric_limits<double>::infinity());

    void setRange(int range_cm);

    double getRange();

    private:
    double no_return_range_ = std::numeric_limits<double>::infinity();
};
