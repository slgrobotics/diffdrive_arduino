See my [Main Project Wiki](https://github.com/slgrobotics/articubot_one/wiki)


**Credits:** Original code by Articulated Robotics (Josh Newans):

- https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros
- https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-real#getting-the-hardware-interface

## `diffdrive_arduino` - a unified ROS2 *base* driver 

- Implements *diffdrive_arduino* package ([ros2_control](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts) interfaces)
- Connects to an Arduino Mega 2560, Teensy 4.x or RPi Pico.
- Uses simple strings-based protocol to communicate to microcontroller, which controls wheels motors and encoders, monitors battery and may control some sensors (e.g. sonars).
- It needs to be cloned on the robots with matching Arduino-style controllers only. No workstation side installs.

> **Note** This "main" branch was updated to work in ROS 2 Lyrical.
> For *Jazzy* and *Kilted* clone it at hash `4d5c6c6c5b0db3c508759deaa5d02cd4f1caf54a` (Oct 5,2026 6:47 PM CST):
> ```
> cd ~/robot_ws/src
> git clone https://github.com/slgrobotics/diffdrive_arduino.git
> cd diffdrive_arduino
> git checkout 4d5c6c6c5b0db3c508759deaa5d02cd4f1caf54a
> ```

Here is an explanation of [how it works](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/MakeYourOwn/README.md#base-control---the-arduino-way).

For matching Arduino IDE **firmware code** see:
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS (Arduino Mega 2560)
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS (Arduino Mega 2560)
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Pico (Raspberry Pi Pico)
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Seggy  (BLDC motor wheels & Teensy 4.x)

Please refer to [my Project Wiki](https://github.com/slgrobotics/articubot_one/wiki) for detailed instructions.

### Linux serial transport

The driver uses a local POSIX serial wrapper (`termios` and `poll`); no `serial`,
LibSerial, or additional serial package is required. Ports use raw 8N1 without
hardware or software flow control. The existing `device`, `baud_rate`, and
`timeout` hardware parameters still apply; `timeout` must be positive and is the
maximum time in milliseconds to receive a complete reply. Common Linux baud rates
from 1200 through 4000000 are supported; unsupported rates fail configuration.

Commands and replies end in `\r`. Each command/reply transaction is synchronized,
writes have a 100 ms deadline, and replies are limited to 1024 bytes. Input left
from a previous transaction is discarded before sending a new command. Input is
never flushed after sending. Incomplete, oversized, or mismatched replies fail
the transaction; I/O errors and write timeouts close the connection. Commands are
not automatically retried. Opening the port may reset the microcontroller;
activation waits one second before its startup handshake.

The firmware protocol has no transaction IDs, so a reply arriving after its
timeout can still be confused with a later request for the same command. Set
`timeout` above the firmware's worst-case response latency.

Pseudo-terminal regression tests cover framing, fragmented responses, bounded
reads and writes, concurrent transactions, protocol validation, and disconnects:

```bash
colcon build --packages-select diffdrive_arduino --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select diffdrive_arduino
colcon test-result --verbose
```

### Data flow

During each control cycle:
1. **Read:** `diffdrive_arduino` requests encoder counts and periodically reads battery and sonar data. It updates the ROS hardware state interfaces.
2. **Update:** ROS 2 `diff_drive_controller` uses wheel feedback to calculate odometry and converts the requested robot motion into wheel commands.
3. **Write:** `diffdrive_arduino` converts those wheel commands into the motor-command values expected by the Arduino firmware and sends them over serial.

The Arduino-style MC handles the motors and physical sensors. *diffdrive_arduino* connects that firmware to ROS 2 through `ros2_control`.

<img alt="data flow" src="https://github.com/user-attachments/assets/3b0e627a-09cb-4c73-b42d-95836b03633c" />

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

-------------------------------------

See my [Main Project Wiki](https://github.com/slgrobotics/articubot_one/wiki)
