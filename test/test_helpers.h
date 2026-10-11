#ifndef DIFFDRIVE_ARDUINO_TEST_HELPERS_H
#define DIFFDRIVE_ARDUINO_TEST_HELPERS_H

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include <stdexcept>
#include <vector>

inline hardware_interface::HardwareInfo hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "test_robot";
  info.type = "system";
  info.hardware_parameters = {
    {"left_wheel_name", "left"}, {"right_wheel_name", "right"},
    {"startup_delay_ms", "0"}, {"timeout", "50"}};
  for (const auto & name : {"left", "right"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    hardware_interface::InterfaceInfo velocity{};
    velocity.name = "velocity";
    hardware_interface::InterfaceInfo position{};
    position.name = "position";
    joint.command_interfaces = {velocity};
    joint.state_interfaces = {position, velocity};
    info.joints.push_back(joint);
  }
  return info;
}

inline double state_value(
  const std::vector<hardware_interface::StateInterface> & states, const std::string & name)
{
  for (const auto & state : states) {
    if (state.get_name() == name) { return state.get_optional<double>().value(); }
  }
  throw std::runtime_error("Missing test state: " + name);
}

#endif
