#pragma once

#include <string>
#include <cstdint>
#include <cmath>

#include "sensor_msgs/msg/battery_state.hpp"

// Typical 48V battery has 13 cells in series (13S1P, 13S3P)
#define MAX_BATTERY_CELLS 15

class Battery
{
    public:

    std::string name = "";
    int num_cells = 0; // can be set if you want State Of Charge (SOC, "percent" calculated by voltage in setPercentage())

    // We want to match https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/BatteryState.html

    // The Raspberry Pi 5's processor is optimized for single-precision floating-point (32-bit "float" vs 64-bit "double")
    // but all hardware interfaces are defined with "double" arguments. So, we have to use "double" here.

    // We have these public (for direct access from interfaces) and don't need getters.
    // all these values should be "double" because of that.
    double voltage {0.0};     // Voltage in Volts (Mandatory)
    double temperature {NAN}; // Temperature in Degrees Celsius (If unmeasured NaN)
    double current {NAN};     // Negative when discharging (A)  (If unmeasured NaN)
    double charge {NAN};      // Current charge in Ah  (If unmeasured NaN)
    double capacity {NAN};    // Capacity in Ah (last full capacity)  (If unmeasured NaN)
    double design_capacity {NAN}; // Capacity in Ah (design capacity)  (If unmeasured NaN)
    double percentage {NAN};  // Charge percentage on 0 to 1 range  (If unmeasured NaN)
    double power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN; // The charging status as reported. Values defined above
    double power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN; // The battery health metric. Values defined above
    double power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN; // The battery chemistry. Values defined above
    double present = 0;   // 1 (True) if the battery is present
    double cell_voltage[MAX_BATTERY_CELLS];    // An array of individual cell voltages for each cell in the pack
                              // If individual voltages unknown but number of cells known set each to NaN
    double cell_temperature[MAX_BATTERY_CELLS];// An array of individual cell temperatures for each cell in the pack
                              // If individual temperatures unknown but number of cells known set each to NaN
    std::string location {""};          // The location into which the battery is inserted. (slot number or plug)
    std::string serial_number {""};     // The best approximation of the battery serial number

    Battery() = default;

    void setup(const std::string &battery_name, int number_of_cells);

    // DesignCapacity, PowerSupplyTechnology, Location, SerialNumber are preset in BatteryStateBroadcaster

    inline void setPowerSupplyTechnology(uint8_t val) { power_supply_technology = val; }

    inline void setDesignCapacity(double design_capacity_ah) { capacity = design_capacity_ah; }

    // Only dynamic values are supplied by the Base driver interfaces.
    // We want setters because of possible conversions to "double":

    inline void setVoltage(int volts) { voltage = volts; }

    inline void setTemperature(double temp_deg_celcius) { temperature = temp_deg_celcius; }

    inline void setCurrent(int current_amps) { current = current_amps; }

    void setCharge(int current_charge_ah);

    void setCapacity(double capacity_ah);

    void setPercentage(); // based on voltage

    inline void setPowerSupplyStatus(uint8_t val) { power_supply_status = val; }

    inline void setPowerSupplyHealth(uint8_t val) { power_supply_health = val; }

    inline void setPresent(bool val) { present = val; }

};
