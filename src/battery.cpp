#include "diffdrive_arduino/battery.h"

#include <cmath>


Battery::Battery(const std::string &battery_name)
{
  setup(battery_name);
}


void Battery::setup(const std::string &battery_name)
{
  name = battery_name;
}

void Battery::setVoltage(int voltage_mv)
{
  voltage = ((double)voltage_mv) / 1000.0; // Volts
}

double Battery::getVoltage()
{
  return voltage;
}

void Battery::setCurrent(int current_ma)
{
  current = ((double)current_ma) / 1000.0; // Amperes
}

double Battery::getCurrent()
{
  return current;
}
