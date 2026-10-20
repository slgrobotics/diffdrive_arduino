#include "diffdrive_arduino/posix_serial.h"
#include "diffdrive_arduino/arduino_comms.h"

#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <future>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

template<class Exception, class Function>
void expectThrow(Function action)
{
  try { action(); }
  catch (const Exception &) { return; }
  throw std::runtime_error("Expected exception was not thrown");
}

struct Terminal
{
  int master = -1;
  std::string device;
  Terminal()
  {
    master = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
    require(master >= 0, "Cannot open pseudo-terminal");
    require(grantpt(master) == 0 && unlockpt(master) == 0, "Cannot initialize pseudo-terminal");
    device = ptsname(master);
  }
  ~Terminal() { if (master >= 0) ::close(master); }
  std::string command()
  {
    std::string result;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (true)
    {
      require(std::chrono::steady_clock::now() < deadline, "Emulator command timed out");
      pollfd p{master, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0) continue;
      char buffer[4096];
      auto count = ::read(master, buffer, sizeof(buffer));
      require(count > 0, "Emulator read failed");
      result.append(buffer, static_cast<size_t>(count));
      if (result.back() == '\r') return result;
    }
  }
  void send(const std::string &data)
  {
    size_t offset = 0;
    while (offset < data.size())
    {
      auto count = ::write(master, data.data() + offset, data.size() - offset);
      require(count > 0, "Emulator write failed");
      offset += static_cast<size_t>(count);
    }
  }
};

int main()
{
  try
  {
    Terminal terminal;
    PosixSerial serial;
    require(!serial.isOpen(), "New transport is open");
    expectThrow<std::runtime_error>([&] { serial.exchange("e\r"); });
    expectThrow<std::invalid_argument>([&] { serial.open(terminal.device, 12345, 100); });
    expectThrow<std::invalid_argument>([&] { serial.open(terminal.device, 57600, 0); });
    serial.open(terminal.device, 57600, 80);
    expectThrow<std::invalid_argument>([&] { serial.exchange(""); });
    expectThrow<std::invalid_argument>([&] { serial.exchange("e"); });
    expectThrow<std::invalid_argument>([&] { serial.exchange("e\rh\r"); });
    int observer = ::open(terminal.device.c_str(), O_RDWR | O_NOCTTY);
    require(observer >= 0, "Cannot inspect terminal");
    termios settings{};
    require(tcgetattr(observer, &settings) == 0, "Cannot inspect serial settings");
    require(cfgetospeed(&settings) == B57600, "Baud rate not applied");
    require((settings.c_cflag & CSIZE) == CS8 && !(settings.c_cflag & (PARENB | CSTOPB | CRTSCTS)), "Not 8N1 without flow control");
    require(!(settings.c_lflag & (ICANON | ECHO)) && !(settings.c_iflag & (IXON | IXOFF | ICRNL)), "Port not raw");
    ::close(observer);

    terminal.send("stale reply\r");
    auto emulator = std::async(std::launch::async, [&] {
      require(terminal.command() == "e\r", "Wrong encoder command");
      terminal.send("e -12");
      std::this_thread::sleep_for(10ms);
      terminal.send(" 34\r");
    });
    require(serial.exchange("e\r") == "e -12 34\r", "Fragmented response lost");
    emulator.get();

    emulator = std::async(std::launch::async, [&] {
      require(terminal.command() == "e\r", "Wrong timeout command");
      terminal.send("e partial");
    });
    const auto begin = std::chrono::steady_clock::now();
    require(serial.exchange("e\r").empty(), "Partial reply accepted");
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    require(elapsed >= 60ms && elapsed < 500ms, "Read deadline not respected");
    emulator.get();

    // Continuous incoming bytes must not extend the overall response deadline.
    emulator = std::async(std::launch::async, [&] {
      terminal.command();
      for (int i = 0; i < 12; ++i) { terminal.send("x"); std::this_thread::sleep_for(10ms); }
    });
    const auto trickle_begin = std::chrono::steady_clock::now();
    require(serial.exchange("e\r").empty(), "Unterminated trickle accepted");
    require(std::chrono::steady_clock::now() - trickle_begin < 500ms, "Trickle extended deadline");
    emulator.get();

    emulator = std::async(std::launch::async, [&] { terminal.command(); terminal.send("12345"); });
    expectThrow<std::length_error>([&] { serial.exchange("e\r", 4); });
    emulator.get();

    // A long command forces partial writes through the pseudo-terminal buffer.
    const std::string long_command = std::string(128 * 1024, 'x') + '\r';
    emulator = std::async(std::launch::async, [&] {
      require(terminal.command() == long_command, "Partial write corrupted command");
      terminal.send("x OK\r");
    });
    require(serial.exchange(long_command) == "x OK\r", "Partial write transaction failed");
    emulator.get();

    // Entire transactions, including replies, are serialized between callers.
    emulator = std::async(std::launch::async, [&] {
      for (int i = 0; i < 2; ++i) {
        const auto request = terminal.command();
        require(request == "e\r" || request == "h\r", "Concurrent commands interleaved");
        std::this_thread::sleep_for(5ms);
        terminal.send(request.substr(0, 1) + " OK\r");
      }
    });
    auto first = std::async(std::launch::async, [&] { return serial.exchange("e\r"); });
    auto second = std::async(std::launch::async, [&] { return serial.exchange("h\r"); });
    require(first.get() == "e OK\r" && second.get() == "h OK\r", "Concurrent replies crossed");
    emulator.get();

    serial.open(terminal.device, 115200, 80); // Reconfiguration closes the old descriptor.
    ArduinoComms arduino;
    arduino.setup(terminal.device, 115200, 80);
    emulator = std::async(std::launch::async, [&] {
      require(terminal.command() == "e\r", "Arduino encoder command changed");
      terminal.send("e -123 456\r");
      require(terminal.command() == "m 10 -20\r", "Motor command changed");
      terminal.send("m OK\r");
      require(terminal.command() == "e\r", "Malformed response test command changed");
      terminal.send("h 1 2\r");
      terminal.command(); terminal.send("e bad\r");
      terminal.command(); terminal.send("e 1 2");
    });
    int left = 0, right = 0;
    require(arduino.readEncoderValues(left, right) && left == -123 && right == 456, "Encoder reply parse failed");
    arduino.setMotorValues(10, -20);
    for (int i = 0; i < 3; ++i) {
      require(!arduino.readEncoderValues(left, right), "Invalid encoder reply accepted");
      require(left == -123 && right == 456, "Invalid reply changed encoder state");
    }
    emulator.get();


    // Firmware-specific handshake and acknowledgment rules.
    emulator = std::async(std::launch::async, [&] {
      require(terminal.command() == "\r", "Startup command changed");
      terminal.send("* OK\r");
      require(terminal.command() == "u 30:20:0:100\r", "PID command changed");
      terminal.send("u OK\r");
      terminal.command(); terminal.send("m ERROR\r");
      terminal.command(); terminal.send("? Error: 'm'\r");
      terminal.command(); // Missing motor acknowledgment.
    });
    arduino.sendEmptyMsg();
    arduino.setPidValues(30, 20, 0, 100);
    for (int i = 0; i < 3; ++i)
      expectThrow<std::runtime_error>([&] { arduino.setMotorValues(0, 0); });
    emulator.get();
    emulator = std::async(std::launch::async, [&] {
      terminal.command(); terminal.send("u ERROR\r");
      terminal.command(); terminal.send("* ERROR\r");
    });
    expectThrow<std::runtime_error>([&] { arduino.setPidValues(30, 20, 0, 100); });
    expectThrow<std::runtime_error>([&] { arduino.sendEmptyMsg(); });
    emulator.get();

    // Firmware numeric telemetry must parse atomically, including signed current.
    const std::string invalid_health[] = {
      "h bogus -1 0\r", "h 12000x -1 0\r", "h 2147483648 -1 0\r",
      "h 12000 -1\r", "h 12000 -1 0 extra\r", "h 12000-1 0\r"};
    const std::string invalid_sonar[] = {
      "p 0 0 bad 0\r", "p 0 0 0\r", "p 0 0 0 2147483648\r", "p 0 0 0 0 extra\r"};
    emulator = std::async(std::launch::async, [&] {
      require(terminal.command() == "h\r", "Health command changed");
      terminal.send("h 12600 -1 0\r");
      require(terminal.command() == "h\r", "Dragger health command changed");
      terminal.send("h 13713 4530 2048\r");
      for (const auto &response : invalid_health) { terminal.command(); terminal.send(response); }
      require(terminal.command() == "p\r", "Sonar command changed");
      terminal.send("p 0 0 0 0\r");
      terminal.command(); terminal.send("p 10 20 30 255\r");
      for (const auto &response : invalid_sonar) { terminal.command(); terminal.send(response); }
    });
    int voltage = 0, current = 0, memory = 7;
    require(arduino.readHealthValues(voltage, current, memory), "Valid health reply rejected");
    require(voltage == 12600 && current == -1 && memory == 0, "Health units/values changed");
    require(arduino.readHealthValues(voltage, current, memory), "Dragger health reply rejected");
    require(voltage == 13713 && current == 4530 && memory == 2048, "Dragger health values changed");
    for (const auto &response : invalid_health) {
      (void)response;
      require(!arduino.readHealthValues(voltage, current, memory), "Malformed health accepted");
      require(voltage == 13713 && current == 4530 && memory == 2048, "Malformed health changed state");
    }
    int fr = -1, fl = -1, br = -1, bl = -1;
    require(arduino.readPingValues(fr, fl, br, bl), "Firmware zero sonar reply rejected");
    require(fr == 0 && fl == 0 && br == 0 && bl == 0, "Zero sonar values changed");
    require(arduino.readPingValues(fr, fl, br, bl), "Valid sonar reply rejected");
    require(fr == 10 && fl == 20 && br == 30 && bl == 255, "Sonar order changed");
    for (const auto &response : invalid_sonar) {
      (void)response;
      require(!arduino.readPingValues(fr, fl, br, bl), "Malformed sonar accepted");
      require(fr == 10 && fl == 20 && br == 30 && bl == 255, "Malformed sonar changed state");
    }
    emulator.get();

    ::close(terminal.master);
    terminal.master = -1;
    expectThrow<std::system_error>([&] { serial.exchange("e\r"); });
    require(!serial.isOpen(), "Disconnected descriptor retained");
    require(arduino.sendMsg("e\r").empty(), "Disconnect not reported as failed transaction");
    require(!arduino.connected(), "Arduino remains connected after I/O failure");

    Terminal blocked;
    serial.open(blocked.device, 115200, 80);
    const auto write_begin = std::chrono::steady_clock::now();
    expectThrow<std::runtime_error>([&] { serial.exchange(long_command); });
    require(std::chrono::steady_clock::now() - write_begin < 500ms, "Blocked write did not time out");
    require(!serial.isOpen(), "Partially written command left connection reusable");

    std::cout << "Serial regression checks passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
