#include "diffdrive_arduino/arduino_comms.h"

#include <algorithm>
#include <chrono>
#include <charconv>
#include <sstream>
#include <stdexcept>

void ArduinoComms::setup(const std::string & device, int baud_rate, int timeout_ms)
{
  if (device.empty() || baud_rate <= 0 || timeout_ms <= 0) {
    throw std::invalid_argument("Invalid serial configuration");
  }
  disconnect();
  timeout_ms_ = timeout_ms;
  serial_conn_.setPort(device);
  serial_conn_.setBaudrate(baud_rate);
  auto timeout = serial::Timeout::simpleTimeout(timeout_ms_);
  serial_conn_.setTimeout(timeout);
  serial_conn_.open();
}

void ArduinoComms::disconnect()
{
  if (connected()) { serial_conn_.close(); }
}

void ArduinoComms::readEncoderValues(int32_t & left, int32_t & right)
{
  std::istringstream stream(sendMsg("e\r"));
  std::string left_text;
  std::string right_text;
  std::string extra;
  if (!(stream >> left_text >> right_text) || (stream >> extra)) {
    throw std::runtime_error("Expected exactly two encoder counts");
  }
  auto parse = [](const std::string & text) {
    int32_t value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc() || result.ptr != text.data() + text.size()) {
      throw std::runtime_error("Invalid or out-of-range encoder count");
    }
    return value;
  };
  const auto new_left = parse(left_text);
  const auto new_right = parse(right_text);
  // Commit both values only after validating the complete response.
  left = new_left;
  right = new_right;
}

void ArduinoComms::setMotorValues(int left, int right)
{
  if (left < -32767 || left > 32767 || right < -32767 || right > 32767) {
    throw std::invalid_argument("Motor command exceeds firmware integer range");
  }
  expectOk("m " + std::to_string(left) + " " + std::to_string(right) + "\r");
}

void ArduinoComms::setPidValues(int p, int d, int i, int o)
{
  const auto values = std::to_string(p) + ":" + std::to_string(d) + ":" +
    std::to_string(i) + ":" + std::to_string(o);
  if (p < 0 || d < 0 || i < 0 || o <= 0 || p > 32767 || d > 32767 || i > 32767 ||
    o > 32767 || values.size() > 15)
  {
    throw std::invalid_argument("Invalid PID values or firmware argument buffer overflow");
  }
  expectOk("u " + values + "\r");
}

bool ArduinoComms::stopMotors() noexcept
{
  try {
    if (!connected()) { return false; }
    auto timeout = serial::Timeout::simpleTimeout(timeout_ms_);
    serial_conn_.setTimeout(timeout);
    // A leading CR terminates any partially transmitted command.
    const std::string stop = "\rm 0 0\r";
    return serial_conn_.write(stop) == stop.size();
  } catch (...) { return false; }
}

void ArduinoComms::expectOk(const std::string & message)
{
  if (sendMsg(message) != "OK") { throw std::runtime_error("Arduino did not acknowledge command"); }
}

std::string ArduinoComms::sendMsg(const std::string & message)
{
  if (!connected()) { throw std::runtime_error("Serial port is not open"); }
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms_);
  auto timeout = serial::Timeout::simpleTimeout(timeout_ms_);
  serial_conn_.setTimeout(timeout);
  serial_conn_.flushInput();
  if (serial_conn_.write(message) != message.size()) {
    throw std::runtime_error("Incomplete serial command write");
  }
  std::string response;
  // readline() applies its timeout per byte. Use one deadline for the entire transaction.
  while (response.size() < 128) {
    const auto remaining = deadline - Clock::now();
    if (remaining <= Clock::duration::zero()) { break; }
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
    timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(std::max<int64_t>(1, milliseconds)));
    serial_conn_.setTimeout(timeout);
    const auto byte = serial_conn_.read(1);
    if (byte.empty()) { continue; }
    if (byte[0] == '\n') {
      if (!response.empty() && response.back() == '\r') { response.pop_back(); }
      return response;
    }
    response += byte;
  }
  throw std::runtime_error("Serial response timed out or exceeded 128 bytes");
}
