#include "diffdrive_arduino/wheel.h"
#include "diffdrive_arduino/encoder_parser.h"
#include <rcpputils/version.h>
#include <diff_drive_controller/odometry.hpp>
#include <cmath>
#include <climits>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

bool near(double actual, double expected)
{
  return std::abs(actual - expected) < 1e-9;
}

int main()
{
  try
  {
    int left = 7, right = 8;
    for (const char *invalid : {"", "1", "garbage", "1 nope", "1 2 extra", "1 2.5",
                               "1-2", "2147483648 0", "0 -2147483649"})
    {
      require(!parseEncoderValues(invalid, left, right), "Malformed encoder reply accepted");
      require(left == 7 && right == 8, "Failed parse modified counters");
    }
    require(parseEncoderValues("0 0\r", left, right), "Short zero reply rejected");
    require(left == 0 && right == 0, "Zero reply parsed incorrectly");
    require(parseEncoderValues(" -12345\t67890\r\n", left, right), "Signed reply rejected");

    Wheel l("left", 60), r("right", 60);
    l.setEncoderBaseline(-12345);
    r.setEncoderBaseline(67890);
    diff_drive_controller::Odometry odometry;
    odometry.setWheelParams(0.473, 0.122, 0.122);
    for (int i = 0; i < 20; ++i)
    {
      l.updateEncoder(-12345, 1.0 / 60);
      r.updateEncoder(67890, 1.0 / 60);
      odometry.update_from_pos(l.pos, r.pos, 1.0 / 60);
      require(near(l.pos, 0) && near(r.pos, 0), "Retained counters produced a position jump");
      require(near(l.vel, 0) && near(r.vel, 0), "Retained counters produced hardware velocity");
      require(near(odometry.getLinear(), 0) && near(odometry.getAngular(), 0),
              "Retained counters produced controller startup velocity");
    }
    l.updateEncoder(-12344, 0.1);
    r.updateEncoder(67891, 0.1);
    odometry.update_from_pos(l.pos, r.pos, 0.1);
    require(near(l.pos, l.rads_per_count) && near(l.vel, l.rads_per_count / 0.1),
            "Real forward motion incorrect");
    require(near(odometry.getX(), 0.122 * l.rads_per_count), "Controller movement incorrect");

    const double saved_position = l.pos;
    l.setEncoderBaseline(1234);
    l.updateEncoder(1234, 0.1);
    require(near(l.pos, saved_position) && near(l.vel, 0), "Reactivation changed position");
    l.updateEncoder(1233, 0.1);
    require(near(l.pos, 0) && l.vel < 0, "Reverse movement incorrect");

    l.setEncoderBaseline(INT_MAX);
    l.updateEncoder(INT_MIN, 0.1);
    require(near(l.pos, l.rads_per_count), "Forward rollover incorrect");
    l.updateEncoder(INT_MAX, 0.1);
    require(near(l.pos, 0), "Reverse rollover incorrect");
    for (double dt : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()})
    {
      bool rejected = false;
      try { l.updateEncoder(0, dt); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected && l.enc == INT_MAX && near(l.pos, 0), "Invalid interval mutated state");
    }
    std::cout << "Encoder parsing, startup, controller odometry, motion, reactivation, rollover and timing passed\n";
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
