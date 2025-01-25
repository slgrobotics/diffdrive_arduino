#pragma once

#include <serial/serial.h>
#include <cstring>

class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  {  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  void sendEmptyMsg();
  
  void readEncoderValues(int &val_1, int &val_2);
  void readHealthValues(int &mv_per_cell, int &current_ma, int &free_mem_bytes);
  void readPingValues(int &front_right, int &front_left, int &back_right, int &back_left);

  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  serial::Serial serial_conn_;  ///< Underlying serial connection 
};
