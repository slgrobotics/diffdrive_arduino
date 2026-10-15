#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

// Linux serial transport for one command / CR-terminated response at a time.
class PosixSerial
{
public:
  PosixSerial() = default;
  ~PosixSerial();
  PosixSerial(const PosixSerial &) = delete;
  PosixSerial &operator=(const PosixSerial &) = delete;

  void open(const std::string &device, int32_t baud_rate, int32_t timeout_ms);
  bool isOpen() const;
  // Returns an empty string on timeout; throws on I/O errors or oversized replies.
  std::string exchange(const std::string &command, size_t max_response = 1024);

private:
  using Clock = std::chrono::steady_clock;
  bool wait(short events, Clock::time_point deadline);
  void closeUnlocked();
  [[noreturn]] void ioError(const char *operation, int error);

  mutable std::mutex mutex_;
  int fd_ = -1;
  int32_t timeout_ms_ = 1000;
};
