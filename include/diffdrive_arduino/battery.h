#pragma once

#include <string>

class Battery
{
    public:

    std::string name = "";
    double voltage = 0.0;

    Battery() = default;

    Battery(const std::string &battery_name);
    
    void setup(const std::string &battery_name);

    double getVoltage();
};
