**Credits:** Original code by Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

## diffdrive_arduino - a unified ROS2 *base* driver 

- Implements *diffdrive_arduino* package ([ros2_control](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts) interfaces)
which connects to an Arduino Mega 2560, Teensy 4.x or RPi Pico.

- Uses simple strings-based protocol to communicate to microcontroller, which controls wheels motors and encoders, monitors battery and may control some sensors (e.g. sonars).

Here is an explanation of [how it works](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/MakeYourOwn/README.md#base-control---the-arduino-way).

For matching Arduino IDE code see:
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Pico
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Seggy  (BLDC motor wheels)

Please refer to [my Project Wiki](https://github.com/slgrobotics/articubot_one/wiki) for detailed instructions.

#### Encoder initialization (code fixes by ChatGPT Codex):

The Arduino may return accumulated signed 32-bit encoder counts from previous runs. 

On hardware activation, the driver requires a valid encoder reply and captures both encoder counts
as baselines. Joint positions start at zero on a fresh ROS2 process and accumulate
only subsequent movement. Hardware reactivation rebases the counters while
preserving existing joint positions. No Arduino counter-reset command is needed.

Encoder replies must contain exactly two integers (`e <left> <right>\r` on the
wire). Invalid, incomplete, or out-of-range replies fail activation or return a
hardware read error without changing wheel state.

Velocity uses elapsed time between valid samples from a monotonic clock. Signed 32-bit rollover is supported;
firmware must not silently reset counters during an active session. If firmware
reboots, deactivate and reactivate the hardware to capture new baselines.

#### Tests
Regression tests cover retained startup counts with the real diff-drive odometry
implementation, forward/reverse movement, reactivation, counter rollover,
malformed replies, and invalid sample intervals:

```bash
colcon build --packages-select diffdrive_arduino --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select diffdrive_arduino
colcon test-result --verbose
```
