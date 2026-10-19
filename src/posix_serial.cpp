#include "diffdrive_arduino/posix_serial.h"

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace
{
speed_t baudConstant(int32_t baud)
{
  switch (baud)
  {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: throw std::invalid_argument("Unsupported serial baud rate: " + std::to_string(baud));
  }
}
}

PosixSerial::~PosixSerial()
{
  closeUnlocked();
}

void PosixSerial::closeUnlocked()
{
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

void PosixSerial::ioError(const char *operation, int error)
{
  closeUnlocked();
  throw std::system_error(error, std::generic_category(), operation);
}

void PosixSerial::open(const std::string &device, int32_t baud_rate, int32_t timeout_ms)
{
  const speed_t baud = baudConstant(baud_rate);
  if (timeout_ms <= 0) throw std::invalid_argument("Serial timeout must be positive");
  std::lock_guard<std::mutex> lock(mutex_);
  closeUnlocked();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) ioError("Opening serial port", errno);

  termios options{};
  if (tcgetattr(fd_, &options) < 0) ioError("Reading serial settings", errno);
  cfmakeraw(&options);
  options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
  options.c_cflag |= CS8 | CLOCAL | CREAD;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, baud) < 0 || cfsetospeed(&options, baud) < 0 ||
      tcsetattr(fd_, TCSANOW, &options) < 0)
    ioError("Configuring serial port", errno);
  if (tcflush(fd_, TCIOFLUSH) < 0) ioError("Clearing serial port on open", errno);
  timeout_ms_ = timeout_ms;
}

bool PosixSerial::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

bool PosixSerial::wait(short events, Clock::time_point deadline)
{
  while (true)
  {
    const auto now = Clock::now();
    if (now >= deadline) return false;
    const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - now).count();
    pollfd descriptor{fd_, events, 0};
    const int result = ::poll(&descriptor, 1, static_cast<int>(remaining));
    if (result < 0)
    {
      if (errno == EINTR) continue;
      ioError("Polling serial port", errno);
    }
    if (result == 0) continue;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
      ioError("Serial port disconnected", EIO);
    if (descriptor.revents & events) return true;
  }
}

std::string PosixSerial::exchange(const std::string &command, size_t max_response)
{
  if (command.empty() || command.back() != '\r' || command.find('\r') != command.size() - 1)
    throw std::invalid_argument("Serial command must contain exactly one terminating CR");
  if (max_response == 0) throw std::invalid_argument("Response size must be positive");
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("Serial port is not open");

  // Discard leftovers BEFORE sending, so a fast reply cannot be flushed away.
  if (tcflush(fd_, TCIFLUSH) < 0) ioError("Clearing stale serial input", errno);
  const auto write_deadline = Clock::now() + std::chrono::milliseconds(100);
  size_t sent = 0;
  while (sent < command.size())
  {
    if (!wait(POLLOUT, write_deadline))
    {
      // A partial command cannot safely be followed by another transaction.
      closeUnlocked();
      throw std::runtime_error("Serial write timed out");
    }
    const ssize_t count = ::write(fd_, command.data() + sent, command.size() - sent);
    if (count < 0)
    {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
      ioError("Writing serial command", errno);
    }
    if (count == 0) ioError("Writing serial command", EIO);
    sent += static_cast<size_t>(count);
  }

  const auto read_deadline = Clock::now() + std::chrono::milliseconds(timeout_ms_);
  std::string response;
  // Read one byte at a time so we never consume bytes after the delimiter.
  while (response.size() < max_response)
  {
    if (!wait(POLLIN, read_deadline)) return {};
    char byte;
    const ssize_t count = ::read(fd_, &byte, 1);
    if (count < 0)
    {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
      ioError("Reading serial response", errno);
    }
    if (count == 0) ioError("Reading serial response", EIO);
    response.push_back(byte);
    if (byte == '\r') return response;
  }
  throw std::length_error("Serial response exceeds maximum length");
}
