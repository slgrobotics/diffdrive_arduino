#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "battery.h"

// The official broadcaster expects percentage on a 0..100 scale and a bool
// presence interface. Keep Battery's internal SOC fraction for charge calculations.
class BatteryInterfaces
{
public:
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> export_interfaces(Battery & battery)
  {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> result;
    auto add = [&](const char * interface, double * value) {
      result.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        battery.name, interface, value));
    };
    add("battery_voltage", &battery.voltage);
    add("battery_temperature", &battery.temperature);
    add("battery_current", &battery.current);
    add("battery_charge", &battery.charge);
    add("battery_percentage", &percentage_);
    add("battery_power_supply_status", &battery.power_supply_status);
    add("battery_power_supply_health", &battery.power_supply_health);
    hardware_interface::InterfaceInfo present_info{};
    present_info.name = "battery_present";
    present_info.data_type = "bool";
    present_info.initial_value = "false";
    present_ = std::make_shared<hardware_interface::StateInterface>(
      hardware_interface::InterfaceDescription(battery.name, present_info));
    result.emplace_back(present_);
    update(battery);
    return result;
  }

  void update(const Battery & battery)
  {
    percentage_ = std::isfinite(battery.percentage) ?
      std::clamp(battery.percentage, 0.0, 1.0) * 100.0 : NAN;
    if (present_) { (void)present_->set_value(battery.present != 0.0); }
  }

private:
  double percentage_ = NAN;
  std::shared_ptr<hardware_interface::StateInterface> present_;
};
