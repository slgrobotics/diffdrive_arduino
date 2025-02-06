#pragma once

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "config.h"
#include "wheel.h"
#include "range.h"
#include "battery.h"
#include "arduino_comms.h"

using hardware_interface::return_type;

class DiffDriveArduino
 : public hardware_interface::SystemInterface
{

public:
  DiffDriveArduino();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  Config cfg_;
  ArduinoComms arduino_;

  Wheel l_wheel_;
  Wheel r_wheel_;

  Range range_f_l_;
  Range range_f_r_;
  Range range_b_l_;
  Range range_b_r_;

  Battery battery_;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;

  int voltage_mv, current_ma, free_mem_bytes;
  int front_right, front_left, back_right, back_left; // centimeters

  int bat_cnt_ = 0;
  int print_cnt_ = 0;
};

