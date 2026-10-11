#include "diffdrive_arduino/fake_robot.h"

#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

FakeRobot::FakeRobot()
: logger_(rclcpp::get_logger("FakeRobot"))
{}

CallbackReturn FakeRobot::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    cfg_ = read_config(info_, false);
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Invalid fake hardware configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


void FakeRobot::clear_commands()
{
  l_wheel_.cmd = r_wheel_.cmd = 0.0;
  l_wheel_.vel = r_wheel_.vel = 0.0;
}

CallbackReturn FakeRobot::on_activate(const rclcpp_lifecycle::State &)
{
  clear_commands();
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FakeRobot::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  clear_commands();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FakeRobot::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return on_deactivate(state);
}

CallbackReturn FakeRobot::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_deactivate(state);
}

CallbackReturn FakeRobot::on_error(const rclcpp_lifecycle::State & state)
{
  return on_deactivate(state);
}

return_type FakeRobot::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double seconds = period.seconds();
  if (!std::isfinite(seconds) || seconds <= 0) {
    clear_commands();
    active_ = false;
    return return_type::ERROR;
  }
  if (active_) {
    const double left = l_wheel_.pos + l_wheel_.vel * seconds;
    const double right = r_wheel_.pos + r_wheel_.vel * seconds;
    if (!std::isfinite(left) || !std::isfinite(right)) {
      clear_commands();
      active_ = false;
      return return_type::ERROR;
    }
    l_wheel_.pos = left;
    r_wheel_.pos = right;
  }
  return return_type::OK;
}

return_type FakeRobot::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) { return return_type::OK; }
  if (!std::isfinite(l_wheel_.cmd) || !std::isfinite(r_wheel_.cmd)) {
    clear_commands();
    active_ = false;
    return return_type::ERROR;
  }
  l_wheel_.vel = l_wheel_.cmd;
  r_wheel_.vel = r_wheel_.cmd;
  return return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(FakeRobot, hardware_interface::SystemInterface)
