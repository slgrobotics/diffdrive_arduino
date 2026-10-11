#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <exception>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"


using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

class DiffDriveArduino : public hardware_interface::SystemInterface
{


public:
  DiffDriveArduino();
  ~DiffDriveArduino() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;



private:

  Config cfg_;
  ArduinoComms arduino_;

  Wheel l_wheel_;
  Wheel r_wheel_;

  rclcpp::Logger logger_;

  bool active_ = false;
  bool fault_ = false;

  void clear_commands();
  void seed_encoders();
  void close_connection() noexcept;
  return_type fail(const std::exception & error);
  
};


#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H