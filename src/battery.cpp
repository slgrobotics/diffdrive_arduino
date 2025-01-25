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

double Battery::getVoltage()
{
  return voltage;
}
