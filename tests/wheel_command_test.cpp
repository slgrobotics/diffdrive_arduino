// Verify firmware-percent deadband boundaries, signs, independent wheels, and stops.
// Run with ctest -R wheel_command_test --output-on-failure after a test-enabled build.
// No hardware is accessed; success returns zero, failed checks print their reason.
#include "diffdrive_arduino/wheel.h"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value, const char *reason)
{
  if (!value) throw std::runtime_error(reason);
}

int main()
{
  try
  {
    Wheel left, right, legacy;
    left.setup("left", 60, 28.5714, 9);
    right.setup("right", 60, 28.5714, 9);
    legacy.setup("legacy", 60, 28.5714);
    for (int command = -100; command <= 100; ++command)
    {
      left.cmd = legacy.cmd = command / 28.5714;
      require(left.getRawCmd() == (std::abs(command) < 9 ? 0 : command),
              "Deadband sign or boundary incorrect");
      require(legacy.getRawCmd() == command, "Disabled deadband changed legacy output");
    }
    // Threshold applies AFTER rounding to the firmware's integer wire value.
    left.cmd = 8.49 / 28.5714;
    require(left.getRawCmd() == 0, "Rounded 8 percent must stop");
    left.cmd = 8.51 / 28.5714;
    require(left.getRawCmd() == 9, "Rounded 9 percent must pass");
    left.cmd = 8.0 / 28.5714;
    right.cmd = -20.0 / 28.5714;
    require(left.getRawCmd() == 0 && right.getRawCmd() == -20, "Wheels not independent");
    for (double value : {0.0, -0.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()})
    {
      left.cmd = value;
      require(left.getRawCmd() == 0, "Zero/nonfinite input must stop");
    }
    left.cmd = 1e20;
    require(left.getRawCmd() == 100, "Positive saturation failed");
    left.cmd = -1e20;
    require(left.getRawCmd() == -100, "Negative saturation failed");
    for (int bad : {-1, 101})
    {
      bool rejected = false;
      try { left.setup("left", 60, 28.5714, bad); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, "Invalid threshold accepted");
    }
    std::cout << "Wheel command deadband checks passed\n";
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
