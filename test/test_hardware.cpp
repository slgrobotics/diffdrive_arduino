#include <gtest/gtest.h>
#include <pty.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "diffdrive_arduino/arduino_comms.h"
#include "diffdrive_arduino/diffdrive_arduino.h"
#include "diffdrive_arduino/fake_robot.h"
#include "test_helpers.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class Emulator
{
public:
  explicit Emulator(std::function<std::string(const std::string &)> handler, int byte_delay_ms = 0)
  : handler_(std::move(handler)), byte_delay_ms_(byte_delay_ms)
  {
    char name[128];
    if (openpty(&master_, &slave_, name, nullptr, nullptr) != 0) {
      throw std::runtime_error("Cannot create serial emulator");
    }
    device = name;
    termios settings{};
    tcgetattr(slave_, &settings);
    cfmakeraw(&settings);
    tcsetattr(slave_, TCSANOW, &settings);
    worker_ = std::thread([this] { run(); });
  }
  ~Emulator()
  {
    unplug();
    close(slave_);
  }
  void unplug()
  {
    running_ = false;
    if (worker_.joinable()) { worker_.join(); }
    if (master_ >= 0) { close(master_); master_ = -1; }
  }
  std::vector<std::string> messages()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }
  std::string device;

private:
  void run()
  {
    std::string pending;
    while (running_) {
      pollfd descriptor{master_, POLLIN, 0};
      if (poll(&descriptor, 1, 10) <= 0 || !(descriptor.revents & POLLIN)) { continue; }
      char buffer[128];
      const auto length = ::read(master_, buffer, sizeof(buffer));
      if (length <= 0) { continue; }
      pending.append(buffer, static_cast<size_t>(length));
      size_t end;
      while ((end = pending.find('\r')) != std::string::npos) {
        const auto command = pending.substr(0, end);
        pending.erase(0, end + 1);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          messages_.push_back(command);
        }
        const auto response = handler_(command);
        if (byte_delay_ms_ == 0) {
          if (!response.empty()) { (void)::write(master_, response.data(), response.size()); }
        } else {
          for (const char byte : response) {
            if (!running_) { break; }
            (void)::write(master_, &byte, 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(byte_delay_ms_));
          }
        }
      }
    }
  }
  int master_ = -1;
  int slave_ = -1;
  std::function<std::string(const std::string &)> handler_;
  int byte_delay_ms_ = 0;
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::mutex mutex_;
  std::vector<std::string> messages_;
};

class HardwareTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::Time time{0};
  rclcpp::Duration period{rclcpp::Duration::from_seconds(0.02)};
  rclcpp_lifecycle::State state;
};

TEST_F(HardwareTest, FakeUsesPeriodAndCannotResumeStaleCommands)
{
  FakeRobot robot;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info();
  ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
  auto states = robot.export_state_interfaces();
  auto commands = robot.export_command_interfaces();
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_TRUE(commands[0].set_value(1.0));
  ASSERT_TRUE(commands[1].set_value(2.0));
  ASSERT_EQ(robot.write(time, period), return_type::OK);
  ASSERT_EQ(robot.read(time, period), return_type::OK);
  EXPECT_DOUBLE_EQ(state_value(states, "left/position"), 0.02);
  EXPECT_DOUBLE_EQ(state_value(states, "right/position"), 0.04);
  ASSERT_EQ(robot.on_deactivate(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.read(time, period), return_type::OK);
  EXPECT_DOUBLE_EQ(state_value(states, "left/position"), 0.02);
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.write(time, period), return_type::OK);
  ASSERT_EQ(robot.read(time, period), return_type::OK);
  EXPECT_DOUBLE_EQ(state_value(states, "left/position"), 0.02);
  ASSERT_TRUE(commands[0].set_value(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(robot.write(time, period), return_type::ERROR);
  EXPECT_DOUBLE_EQ(state_value(states, "left/velocity"), 0.0);
}

TEST_F(HardwareTest, FakeRejectsZeroPeriod)
{
  FakeRobot robot;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info();
  ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
  EXPECT_EQ(robot.read(time, rclcpp::Duration::from_seconds(0)), return_type::ERROR);
}

TEST(SerialProtocol, InvalidRepliesNeverOverwriteEitherEncoder)
{
  for (const auto & reply : {"\n", "OK\r\n", "123\n", "1 junk\n", "1 2 3\n",
      "2147483648 2\n", "1 -2147483649\n", "1 2partial\n", "1+2\n", "1 2"})
  {
    Emulator emulator([reply](const auto &) { return std::string(reply); });
    ArduinoComms comms;
    comms.setup(emulator.device, 57600, 30);
    int32_t left = 10;
    int32_t right = 20;
    EXPECT_THROW(comms.readEncoderValues(left, right), std::exception) << reply;
    EXPECT_EQ(left, 10);
    EXPECT_EQ(right, 20);
  }
}

TEST(SerialProtocol, AcceptsSignedCountsAndRejectsBadAcknowledgements)
{
  Emulator emulator([](const auto & command) {
    return command == "e" ? "-2147483648 2147483647\r\n" : "Invalid Command\r\n";
  });
  ArduinoComms comms;
  comms.setup(emulator.device, 57600, 50);
  int32_t left = 0;
  int32_t right = 0;
  ASSERT_NO_THROW(comms.readEncoderValues(left, right));
  EXPECT_EQ(left, std::numeric_limits<int32_t>::min());
  EXPECT_EQ(right, std::numeric_limits<int32_t>::max());
  EXPECT_THROW(comms.setMotorValues(1, 2), std::runtime_error);
}

TEST(SerialProtocol, MissingReplyHasBoundedTimeout)
{
  Emulator emulator([](const auto &) { return std::string(); });
  ArduinoComms comms;
  comms.setup(emulator.device, 57600, 30);
  auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(comms.setMotorValues(1, 2), std::runtime_error);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, std::chrono::milliseconds(200));
}

TEST_F(HardwareTest, ArduinoSeedsStopsAndPreservesPositionAcrossReactivation)
{
  std::atomic<int32_t> count{100000};
  Emulator emulator([&](const auto & command) {
    return command == "e" ? std::to_string(count.load()) + " 100000\r\n" : "OK\r\n";
  });
  DiffDriveArduino robot;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info();
  params.hardware_info.hardware_parameters["device"] = emulator.device;
  ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
  auto states = robot.export_state_interfaces();
  auto commands = robot.export_command_interfaces();
  ASSERT_EQ(robot.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.read(time, period), return_type::OK);
  EXPECT_DOUBLE_EQ(state_value(states, "left/position"), 0.0);
  EXPECT_DOUBLE_EQ(state_value(states, "left/velocity"), 0.0);
  count += 10;
  ASSERT_EQ(robot.read(time, period), return_type::OK);
  const double position = state_value(states, "left/position");
  EXPECT_GT(position, 0.0);
  EXPECT_NEAR(state_value(states, "left/velocity"), position / 0.02, 1e-12);
  ASSERT_TRUE(commands[0].set_value(1.0));
  ASSERT_EQ(robot.write(time, period), return_type::OK);
  ASSERT_EQ(robot.on_deactivate(state), CallbackReturn::SUCCESS);
  const auto messages = emulator.messages();
  ASSERT_FALSE(messages.empty());
  EXPECT_EQ(messages.back(), "m 0 0");
  count = 0;  // A firmware restart while stopped must not move odometry backwards.
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.write(time, period), return_type::OK);
  EXPECT_EQ(emulator.messages().back(), "m 0 0");
  ASSERT_EQ(robot.read(time, period), return_type::OK);
  EXPECT_DOUBLE_EQ(state_value(states, "left/position"), position);
  EXPECT_DOUBLE_EQ(state_value(states, "left/velocity"), 0.0);
  EXPECT_EQ(robot.on_cleanup(state), CallbackReturn::SUCCESS);
}

TEST_F(HardwareTest, ArduinoFaultDoesNotPublishCorruptCountsAndRequiresReconfiguration)
{
  std::atomic<bool> malformed{false};
  Emulator emulator([&](const auto & command) {
    if (command == "e") { return malformed.load() ? "OK\n" : "100000 100000\n"; }
    return "OK\n";
  });
  DiffDriveArduino robot;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info();
  params.hardware_info.hardware_parameters["device"] = emulator.device;
  ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
  auto states = robot.export_state_interfaces();
  ASSERT_EQ(robot.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  malformed = true;
  EXPECT_EQ(robot.read(time, period), return_type::ERROR);
  EXPECT_DOUBLE_EQ(state_value(states, "left/position"), 0.0);
  EXPECT_EQ(robot.write(time, period), return_type::ERROR);
  EXPECT_EQ(robot.on_activate(state), CallbackReturn::ERROR);
  malformed = false;
  ASSERT_EQ(robot.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(robot.read(time, period), return_type::OK);
}

TEST_F(HardwareTest, ArduinoRejectsResetAndNonfiniteOrExcessiveCommands)
{
  for (int scenario = 0; scenario < 3; ++scenario) {
    std::atomic<int32_t> count{100000};
    Emulator emulator([&](const auto & command) {
      return command == "e" ? std::to_string(count.load()) + " 100000\n" : "OK\n";
    });
    DiffDriveArduino robot;
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = hardware_info();
    params.hardware_info.hardware_parameters["device"] = emulator.device;
    ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
    auto states = robot.export_state_interfaces();
    auto commands = robot.export_command_interfaces();
    ASSERT_EQ(robot.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
    if (scenario == 0) {
      count = 0;
      EXPECT_EQ(robot.read(time, period), return_type::ERROR);
      EXPECT_DOUBLE_EQ(state_value(states, "left/position"), 0.0);
    } else {
      ASSERT_TRUE(commands[0].set_value(scenario == 1 ?
        std::numeric_limits<double>::infinity() : 1e100));
      EXPECT_EQ(robot.write(time, period), return_type::ERROR);
    }
    EXPECT_EQ(robot.write(time, period), return_type::ERROR);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto messages = emulator.messages();
    ASSERT_FALSE(messages.empty());
    EXPECT_EQ(messages.back(), "m 0 0");
  }
}

TEST_F(HardwareTest, MissingDeviceReturnsErrorWithoutThrowing)
{
  DiffDriveArduino robot;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info();
  params.hardware_info.hardware_parameters["device"] = "/dev/no-such-diffdrive-test-device";
  ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
  EXPECT_EQ(robot.on_configure(state), CallbackReturn::ERROR);
}

TEST(SerialProtocol, SlowDripCannotExtendTransactionDeadline)
{
  Emulator emulator([](const auto &) { return "1234567890 1234567890\n"; }, 10);
  ArduinoComms comms;
  comms.setup(emulator.device, 57600, 30);
  int32_t left = 10;
  int32_t right = 20;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(comms.readEncoderValues(left, right), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(150));
  EXPECT_EQ(left, 10);
  EXPECT_EQ(right, 20);
}

TEST_F(HardwareTest, UnplugReturnsErrorAndClearsCommands)
{
  Emulator emulator([](const auto & command) { return command == "e" ? "10 20\n" : "OK\n"; });
  DiffDriveArduino robot;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info();
  params.hardware_info.hardware_parameters["device"] = emulator.device;
  ASSERT_EQ(robot.on_init(params), CallbackReturn::SUCCESS);
  auto commands = robot.export_command_interfaces();
  auto states = robot.export_state_interfaces();
  ASSERT_EQ(robot.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(robot.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_TRUE(commands[0].set_value(1.0));
  emulator.unplug();
  EXPECT_EQ(robot.write(time, period), return_type::ERROR);
  EXPECT_DOUBLE_EQ(state_value(states, "left/velocity"), 0.0);
  EXPECT_EQ(robot.write(time, period), return_type::ERROR);
}
