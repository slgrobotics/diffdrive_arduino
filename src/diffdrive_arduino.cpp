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

  // Set up the wheels:
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the ultrasonic sensors:
  range_f_l_.setup("sonar_F_L");
  range_f_r_.setup("sonar_F_R");
  range_b_l_.setup("sonar_B_L");
  range_b_r_.setup("sonar_B_R");

  // Set up the Batery:
  battery_.setup("battery");

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

  state_interfaces.emplace_back(hardware_interface::StateInterface(range_f_l_.name, "range", &range_f_l_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_f_r_.name, "range", &range_f_r_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_b_l_.name, "range", &range_b_l_.range));
  state_interfaces.emplace_back(hardware_interface::StateInterface(range_b_r_.name, "range", &range_b_r_.range));

  state_interfaces.emplace_back(hardware_interface::StateInterface(battery_.name, "battery", &battery_.voltage));

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

  if((++bat_cnt_) > 10)
  {
    bat_cnt_ = 0;

    arduino_.readHealthValues(battery_mv, current_ma, free_mem_bytes);

    battery_.voltage = ((double)battery_mv) / 1000.0;
  }

  arduino_.readPingValues(front_right, front_left, back_right, back_left);

  range_f_l_.range = ((double)front_left) / 100.0; // meters
  range_f_r_.range = ((double)front_right) / 100.0;
  range_b_l_.range = ((double)back_left) / 100.0;
  range_b_r_.range = ((double)back_right) / 100.0;

  if((++print_cnt_) > 100)
  {
    print_cnt_ = 0;

    RCLCPP_INFO(logger_, "Battery Health: %.2f V     %d mA    %d mem bytes free", battery_.getVoltage(), current_ma, free_mem_bytes);

    RCLCPP_INFO(logger_, "Ping: %.2f   %.2f   %.2f   %.2f meters", range_f_l_.getRange(), range_f_r_.getRange(), range_b_l_.getRange(), range_b_r_.getRange());
  }

  return return_type::OK;
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
