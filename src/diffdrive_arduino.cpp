#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"

// See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger
//     https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}


hardware_interface::CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration - Arduino on %s at %d baud", const_cast<char*>(cfg_.device.c_str()), cfg_.baud_rate);

  return CallbackReturn::SUCCESS;
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

/*
this should be in Broadcaster
  // See https://answers.ros.org/question/414029/access-ros-node-in-hardware_interface/
  //     https://github.com/ipa320/ros_battery_monitoring

controller_interface::CallbackReturn DiffDriveArduino::on_configure(const rclcpp_lifecycle::State & / *previous_state* /)
{
  RCLCPP_INFO(logger_, "Configuring Arduino Controller...");

  battery_state_pub_ = getNode()->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", rclcpp::SystemDefaultsQoS());

  realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>(battery_state_pub_);

  realtime_publisher_->msg_.temperature = std::numeric_limits<double>::quiet_NaN();
  realtime_publisher_->msg_.current = std::numeric_limits<double>::quiet_NaN();
  realtime_publisher_->msg_.charge = std::numeric_limits<double>::quiet_NaN();
  realtime_publisher_->msg_.capacity = std::numeric_limits<double>::quiet_NaN();
  realtime_publisher_->msg_.design_capacity = std::numeric_limits<double>::quiet_NaN();
  realtime_publisher_->msg_.percentage = std::numeric_limits<double>::quiet_NaN();
  realtime_publisher_->msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  realtime_publisher_->msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  realtime_publisher_->msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  realtime_publisher_->msg_.present = true;

  int64_t psu_tech = get_node()->get_parameter("power_supply_technology").as_int();
  if (psu_tech != -1)
  {
    realtime_publisher_->msg_.power_supply_technology = psu_tech;
  }

  double design_capacity = get_node()->get_parameter("design_capacity").as_double();
  if (design_capacity != 0.0)
  {
    realtime_publisher_->msg_.design_capacity = static_cast<float>(design_capacity);
  }

  return CallbackReturn::SUCCESS;
}
*/

hardware_interface::CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Arduino Controller...");

  arduino_.sendEmptyMsg();
  //arduino_.sendEmptyMsg();
  //arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  sleep(1);
  arduino_.setPidValues(30, 20, 0, 100);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Arduino Controller...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /*
   * param[in] time The time at the start of this control loop iteration
   * param[in] period The measured time taken by the last control loop iteration
   * return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  //RCLCPP_INFO(logger_, "enc: %d  %d", l_wheel_.enc, r_wheel_.enc);

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds; // (double)period.nanoseconds();

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds; //(double)period.nanoseconds();

  publishBatteryState();

  //int front_right, front_left, back_right, back_left; // centimeters

  //arduino_.readPingValues(front_right, front_left, back_right, back_left);

  //RCLCPP_INFO(logger_, "Ping: %d  %d %d  %d", front_right, front_left, back_right, back_left);

  //std::string gps_values_str;

  //arduino_.readGpsValues(gps_values_str);

  //RCLCPP_INFO(logger_, "GPS: %s", gps_values_str.c_str());

  return return_type::OK;
}

void DiffDriveArduino::publishBatteryState()
{
  if((++bat_cnt_) > 100)
  {
    bat_cnt_ = 0;

    int battery_mv, current_ma, free_mem_bytes;

    arduino_.readHealthValues(battery_mv, current_ma, free_mem_bytes);

    RCLCPP_INFO(logger_, "Battery Health: %d mV     %d mA    %d bytes free", battery_mv, current_ma, free_mem_bytes);

    //auto message = sensor_msgs::msg::BatteryState();

    //battery_state_pub_->publish(message);
  }
}

hardware_interface::return_type DiffDriveArduino::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  //RCLCPP_INFO(logger_, "cmd: %f  sending: %f", l_wheel_.cmd, l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate);

  // Desired speed is set by Comm (Rpi -> Arduino)
  // comes in the range -100...100 - it has a meaning of "percent of max possible speed":
  // For Plucky full wheel rotation at full power takes 3.8 seconds.
  // So, with R_wheel = 0.192m max speed is:
  //   - 0.317 m/sec
  //   - 1.65 rad/sec
  //   - 660 encoder ticks/sec

  // With ROS2 Joystick at full forward:
  //   Normal mode: cmd_vel.linear.x=0.7, l_cmd=50
  //   Turbo mode:  cmd_vel.linear.x=1.4, l_cmd=107

  double wheels_pwm_factor = 1.0;  // account for Plucky wheels "m pwm pwm" command. Turbo joystick mode sends 107 to wheels. 

  double l_cmd = l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate * wheels_pwm_factor;
  double r_cmd = r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate * wheels_pwm_factor;

  //RCLCPP_INFO(logger_, "cmd: %f  sending: %f", l_wheel_.cmd, l_cmd);

  arduino_.setMotorValues(l_cmd, r_cmd);

  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
