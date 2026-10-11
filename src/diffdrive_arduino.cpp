#include "diffdrive_arduino/diffdrive_arduino.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

DiffDriveArduino::DiffDriveArduino()
: logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

DiffDriveArduino::~DiffDriveArduino()
{
  close_connection();
}

CallbackReturn DiffDriveArduino::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    cfg_ = read_config(info_, true);
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Invalid hardware configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


void DiffDriveArduino::clear_commands()
{
  l_wheel_.cmd = r_wheel_.cmd = 0.0;
  l_wheel_.vel = r_wheel_.vel = 0.0;
}

void DiffDriveArduino::seed_encoders()
{
  int32_t left = 0;
  int32_t right = 0;
  arduino_.readEncoderValues(left, right);
  l_wheel_.seed_encoder(left);
  r_wheel_.seed_encoder(right);
}

void DiffDriveArduino::close_connection() noexcept
{
  active_ = false;
  clear_commands();
  // A lost connection cannot guarantee a stop; the firmware watchdog is still required.
  arduino_.stopMotors();
  try { arduino_.disconnect(); }
  catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Serial disconnect failed: %s", error.what());
  }
}

return_type DiffDriveArduino::fail(const std::exception & error)
{
  RCLCPP_ERROR(logger_, "Hardware fault: %s", error.what());
  fault_ = true;
  close_connection();
  return return_type::ERROR;
}

CallbackReturn DiffDriveArduino::on_configure(const rclcpp_lifecycle::State &)
{
  clear_commands();
  active_ = false;
  try {
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
    // Opening a USB serial port can reset the Arduino. Wait outside the control loop.
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.startup_delay_ms));
    arduino_.setMotorValues(0, 0);
    seed_encoders();
    fault_ = false;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    fail(error);
    return CallbackReturn::ERROR;
  }
}

CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State &)
{
  clear_commands();
  try {
    if (fault_ || !arduino_.connected()) {
      throw std::runtime_error("Hardware must be configured before activation");
    }
    arduino_.setMotorValues(0, 0);
    arduino_.setPidValues(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
    seed_encoders();
    active_ = true;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    fail(error);
    return CallbackReturn::ERROR;
  }
}

CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  clear_commands();
  try {
    arduino_.setMotorValues(0, 0);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    fail(error);
    return CallbackReturn::ERROR;
  }
}

CallbackReturn DiffDriveArduino::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  clear_commands();
  try {
    if (arduino_.connected()) { arduino_.setMotorValues(0, 0); }
    arduino_.disconnect();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    fail(error);
    return CallbackReturn::ERROR;
  }
}

CallbackReturn DiffDriveArduino::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

CallbackReturn DiffDriveArduino::on_error(const rclcpp_lifecycle::State &)
{
  fault_ = true;
  close_connection();
  // Return to UNCONFIGURED. Recovery requires an explicit configure/activate cycle.
  return CallbackReturn::SUCCESS;
}

return_type DiffDriveArduino::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  try {
    if (fault_ || !arduino_.connected()) { throw std::runtime_error("Serial hardware is unavailable"); }
    const double seconds = period.seconds();
    if (!std::isfinite(seconds) || seconds <= 0) {
      throw std::runtime_error("Read period must be finite and positive");
    }
    int32_t left = 0;
    int32_t right = 0;
    arduino_.readEncoderValues(left, right);
    if (std::abs(l_wheel_.encoder_delta(left)) > cfg_.max_encoder_delta ||
      std::abs(r_wheel_.encoder_delta(right)) > cfg_.max_encoder_delta)
    {
      throw std::runtime_error("Encoder discontinuity: possible reset or implausible motion");
    }
    const double left_delta = l_wheel_.encoder_delta(left) * l_wheel_.rads_per_count;
    const double right_delta = r_wheel_.encoder_delta(right) * r_wheel_.rads_per_count;
    if (!std::isfinite(l_wheel_.pos + left_delta) || !std::isfinite(r_wheel_.pos + right_delta) ||
      !std::isfinite(left_delta / seconds) || !std::isfinite(right_delta / seconds))
    {
      throw std::runtime_error("Encoder update would produce nonfinite state");
    }
    // Validate both samples before updating either wheel.
    l_wheel_.update_encoder(left, seconds);
    r_wheel_.update_encoder(right, seconds);
    return return_type::OK;
  } catch (const std::exception & error) { return fail(error); }
}

return_type DiffDriveArduino::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fault_) { return return_type::ERROR; }
  if (!active_) { return return_type::OK; }
  try {
    const double left = l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate;
    const double right = r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate;
    if (!std::isfinite(left) || !std::isfinite(right) ||
      std::abs(left) > cfg_.max_motor_command || std::abs(right) > cfg_.max_motor_command)
    {
      throw std::runtime_error("Nonfinite or out-of-range wheel command");
    }
    arduino_.setMotorValues(static_cast<int>(std::round(left)), static_cast<int>(std::round(right)));
    return return_type::OK;
  } catch (const std::exception & error) { return fail(error); }
}

PLUGINLIB_EXPORT_CLASS(DiffDriveArduino, hardware_interface::SystemInterface)
