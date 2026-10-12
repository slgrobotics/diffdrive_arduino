#include <cmath>
#include <stdexcept>
#include "diffdrive_arduino/battery_interfaces.h"

int main()
{
  auto require = [](bool ok) { if (!ok) throw std::runtime_error("battery interface regression"); };
  Battery battery;
  battery.name = "main_battery";
  BatteryInterfaces bridge;
  auto interfaces = bridge.export_interfaces(battery);
  require(interfaces.size() == 8);
  require(interfaces[0]->get_name() == "main_battery/battery_voltage");
  require(std::isnan(interfaces[4]->get_optional<double>().value()));
  require(!interfaces[7]->get_optional<bool>().value());
  battery.percentage = 0.5;
  battery.present = 1;
  battery.voltage = 12.5;
  bridge.update(battery);
  require(interfaces[0]->get_optional<double>().value() == 12.5);
  require(interfaces[4]->get_optional<double>().value() == 50.0);
  require(interfaces[7]->get_optional<bool>().value());
  require(battery.percentage == 0.5);
  battery.percentage = 1.2;
  battery.present = 0;
  bridge.update(battery);
  require(interfaces[4]->get_optional<double>().value() == 100.0);
  require(!interfaces[7]->get_optional<bool>().value());
  battery.percentage = NAN;
  bridge.update(battery);
  require(std::isnan(interfaces[4]->get_optional<double>().value()));
}
