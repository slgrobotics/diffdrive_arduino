#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <cstdint>
#include <string>
#include <serial/serial.h>

class ArduinoComms
{
public:
  void setup(const std::string & device, int baud_rate, int timeout_ms);
  void disconnect();
  void readEncoderValues(int32_t & left, int32_t & right);
  void setMotorValues(int left, int right);
  void setPidValues(int p, int d, int i, int o);
  bool connected() const { return serial_conn_.isOpen(); }
  // Best effort stop without waiting for a reply on an already faulty connection.
  bool stopMotors() noexcept;

private:
  std::string sendMsg(const std::string & message);
  void expectOk(const std::string & message);
  serial::Serial serial_conn_;
  int timeout_ms_ = 10;
};

#endif
