#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "diffdrive_arduino/config.h"
#include "diffdrive_arduino/wheel.h"
#include "test_helpers.h"

TEST(Encoder, ExistingCountsDoNotBecomeMovement)
{
  Wheel wheel("left", 1920);
  wheel.seed_encoder(100000);
  wheel.update_encoder(100000, 0.02);
  EXPECT_DOUBLE_EQ(wheel.pos, 0.0);
  EXPECT_DOUBLE_EQ(wheel.vel, 0.0);
  wheel.update_encoder(100010, 0.02);
  const double angle = 10 * 2 * std::acos(-1.0) / 1920;
  EXPECT_NEAR(wheel.pos, angle, 1e-12);
  EXPECT_NEAR(wheel.vel, angle / 0.02, 1e-12);
  wheel.seed_encoder(0);  // Firmware reset while inactive.
  wheel.update_encoder(0, 0.02);
  EXPECT_NEAR(wheel.pos, angle, 1e-12);
  EXPECT_DOUBLE_EQ(wheel.vel, 0.0);
}

TEST(Encoder, SignedRolloverIsContinuousInBothDirections)
{
  Wheel wheel("left", 1920);
  wheel.seed_encoder(std::numeric_limits<int32_t>::max() - 1);
  const auto next = std::numeric_limits<int32_t>::min() + 1;
  EXPECT_EQ(wheel.encoder_delta(next), 3);
  wheel.update_encoder(next, 0.02);
  EXPECT_NEAR(wheel.pos, 3 * wheel.rads_per_count, 1e-12);
  EXPECT_EQ(wheel.encoder_delta(std::numeric_limits<int32_t>::max() - 1), -3);
}

TEST(Encoder, BadResolutionAndPeriodsCannotCorruptState)
{
  EXPECT_THROW(Wheel("left", 0), std::invalid_argument);
  EXPECT_THROW(Wheel("left", -1), std::invalid_argument);
  Wheel wheel("left", 1920);
  wheel.seed_encoder(100);
  for (double seconds : {0.0, -0.02, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(wheel.update_encoder(101, seconds), std::invalid_argument);
    EXPECT_EQ(wheel.enc, 100);
    EXPECT_DOUBLE_EQ(wheel.pos, 0.0);
  }
}

TEST(Configuration, DefaultsAndFirmwareRate)
{
  auto info = hardware_info();
  auto cfg = read_config(info, true);
  EXPECT_EQ(cfg.max_encoder_delta, cfg.enc_counts_per_rev);
  EXPECT_DOUBLE_EQ(cfg.loop_rate, 30.0);
  info.hardware_parameters["loop_rate"] = "50";
  EXPECT_DOUBLE_EQ(read_config(info, true).loop_rate, 50.0);
}

TEST(Configuration, RejectsInvalidNumbersAndFirmwareBufferOverflow)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"enc_counts_per_rev", "0"}, {"enc_counts_per_rev", "1920junk"},
    {"loop_rate", "0"}, {"loop_rate", "nan"}, {"loop_rate", "inf"},
    {"loop_rate", "30x"}, {"baud_rate", "0"}, {"timeout", "-1"},
    {"startup_delay_ms", "-1"}, {"max_encoder_delta", "0"},
    {"max_motor_command", "32768"}, {"pid_o", "0"}, {"device", ""}};
  for (const auto & item : cases) {
    auto info = hardware_info();
    info.hardware_parameters[item.first] = item.second;
    EXPECT_THROW(read_config(info, true), std::exception) << item.first;
  }
  auto info = hardware_info();
  for (const auto & key : {"pid_p", "pid_d", "pid_i", "pid_o"}) {
    info.hardware_parameters[key] = "32767";
  }
  EXPECT_THROW(read_config(info, true), std::invalid_argument);
}

TEST(Configuration, RejectsInconsistentWheelInterfaces)
{
  auto info = hardware_info();
  info.hardware_parameters["right_wheel_name"] = "left";
  EXPECT_THROW(read_config(info, false), std::invalid_argument);
  info = hardware_info();
  info.hardware_parameters.erase("left_wheel_name");
  EXPECT_THROW(read_config(info, false), std::invalid_argument);
  info = hardware_info();
  info.joints[0].state_interfaces.pop_back();
  EXPECT_THROW(read_config(info, true), std::invalid_argument);
  info = hardware_info();
  info.joints[0].command_interfaces[0].name = "position";
  EXPECT_THROW(read_config(info, true), std::invalid_argument);
}
