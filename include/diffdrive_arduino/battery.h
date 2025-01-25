#pragma once

#include <string>

class Battery
{
    public:

    std::string name = "";

    double voltage = 0.0;   // Volts
    double current = 0.0;   // Amperes

    Battery() = default;

    Battery(const std::string &battery_name);
    
    void setup(const std::string &battery_name);

    void setVoltage(int voltage_mv);

    double getVoltage();

    void setCurrent(int current_ma);

    double getCurrent();
};
