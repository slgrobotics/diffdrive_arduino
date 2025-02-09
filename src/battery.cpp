#include "diffdrive_arduino/battery.h"
#include <cmath>

void Battery::setup(const std::string &battery_name, int number_of_cells)
{
  name = battery_name;
  num_cells = number_of_cells;
}

void Battery::setCharge(int current_charge_ah)
{
  charge = current_charge_ah; // Amp-Hours 
}

void Battery::setCapacity(double capacity_ah)
{
  capacity = capacity_ah; // Amp-Hours
}

void Battery::setPercentage() // based on voltage
{
  // See https://www.jackery.com/blogs/knowledge/ultimate-guide-to-lifepo4-voltage-chart

  switch((int)round(power_supply_technology)) {

    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH: // 1    # Nickel-Metal Hydride battery
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION: // 2    # Lithium-ion battery
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO: // 3    # Lithium Polymer battery
      break;

    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE: // 4    # Lithium Iron Phosphate battery
      {
        double rest_voltage = 3.40 * num_cells;
        double discharged_voltage = 2.50 * num_cells;
        percentage = (voltage - discharged_voltage) / (rest_voltage - discharged_voltage); // must be in the 0...1.0 range
      }
      break;

    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD: // 5    # Nickel-Cadmium battery
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN: // 6    # Lithium Manganese Dioxide battery
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_TERNARY: // 7 # Ternary Lithium battery
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA: // 8    # Valve Regulated Lead-Acid battery
      break;
  }
}
