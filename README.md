See my [Main Project Wiki](https://github.com/slgrobotics/articubot_one/wiki)


**Credits:** Original code by Articulated Robotics (Josh Newans):

- https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros
- https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-real#getting-the-hardware-interface

## Contents

- [Overview](#diffdrive_arduino---a-unified-ros2-base-driver)
- [Hardware parameters and `cfg_`](#hardware-parameters-and-cfg_)
- [Linux serial transport](#linux-serial-transport)
- [Data flow](#data-flow)
- [Encoder initialization](#encoder-initialization-code-fixes-by-chatgpt-codex)
- [Tests](#tests)
- [Optional finite sonar no-return distance](#optional-finite-sonar-no-return-distance)
- [Minimal on-robot tracer (Lyrical)](#minimal-on-robot-tracer-lyrical)

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

### Hardware parameters and `cfg_`

Hardware settings are specified in the robot's URDF/Xacro under
`<ros2_control><hardware><param>`. They are separate from ordinary ROS node
parameters and the controller parameters in YAML.

```text
URDF/Xacro <hardware><param> values
  -> ros2_control HardwareInfo
  -> info_.hardware_parameters (string values)
  -> DiffDriveArduino::on_init()
  -> cfg_ and battery configuration
```

The base-class `SystemInterface::on_init(params)` populates `info_`. The driver's
`on_init()` then copies strings and converts numeric values using `std::stoi()`
(integers) or `std::stof()` (floating point). For example:

```cpp
cfg_.device = info_.hardware_parameters["device"];
cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
```

All **12 hardware parameters below are required by the current initialization
code**, including `loop_rate`, which is parsed but currently unused. Battery
parameters configure `battery_` directly rather than becoming members of `cfg_`.

| Parameter | Type | Meaning |
| --- | --- | --- |
| `left_wheel_name` | String | Left wheel joint name; must match the robot's joint definition. |
| `right_wheel_name` | String | Right wheel joint name; must match the robot's joint definition. |
| `loop_rate` | Float | Nominal loop rate in Hz; currently stored but unused. Set the actual control rate through controller manager's `update_rate`. |
| `device` | String | Linux serial device path, such as `/dev/ttyACM0` or a stable `/dev/serial/by-id/...` path. |
| `baud_rate` | Integer | Serial baud rate; `115200` for the Seggy and Dragger firmware described here. |
| `timeout` | Integer | Positive serial response timeout in milliseconds. |
| `enc_counts_per_rev` | Integer | Encoder counts per complete wheel revolution, accounting for gearing and the firmware's counting mode; must be positive. |
| `wheels_pwm_factor` | Float | Converts wheel velocity commands in rad/s to firmware motor setpoints in percent: `round(velocity * factor)`, clamped to `-100..100`. Calibrate for each robot. |
| `battery_name` | String | Name used for the exported battery sensor interfaces. |
| `battery_num_cells` | Integer | Number of cells in series; used by the battery voltage/charge estimate. |
| `battery_technology` | Integer | A `sensor_msgs/msg/BatteryState` `POWER_SUPPLY_TECHNOLOGY_*` value, e.g. `4` for LiFePO4. |
| `battery_design_capacity` | Float | Battery design capacity in ampere-hours (Ah). |

For example, the hardware block can use robot-specific Xacro properties for the
calibration and battery values (define these properties elsewhere in your Xacro):

```xml
<ros2_control name="ArduinoSystem" type="system">
  <hardware>
    <plugin>diffdrive_arduino/DiffDriveArduino</plugin>
    <param name="left_wheel_name">${left_wheel_joint_name}</param>
    <param name="right_wheel_name">${right_wheel_joint_name}</param>
    <param name="loop_rate">${control_loop_rate}</param>
    <param name="device">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
    <param name="timeout">1000</param>
    <param name="enc_counts_per_rev">${encoder_counts_per_wheel_rev}</param>
    <param name="wheels_pwm_factor">${wheel_velocity_to_percent}</param>
    <param name="battery_name">battery</param>
    <param name="battery_num_cells">${battery_cells_in_series}</param>
    <param name="battery_technology">${battery_chemistry_id}</param>
    <param name="battery_design_capacity">${battery_capacity_ah}</param>
  </hardware>
  <!-- Add the wheel joint command/state interface definitions here. -->
</ros2_control>
```

See actual configuration file example [here](https://github.com/slgrobotics/articubot_one/blob/lyrical/robots/seggy/description/ros2_control.xacro)
and [here](https://github.com/slgrobotics/articubot_one/blob/lyrical/robots/seggy/config/controllers.yaml).

The initializers in `include/diffdrive_arduino/config.h` are **not fallbacks**:
`on_init()` overwrites every `cfg_` field unconditionally. A missing map entry
produces an empty string; string fields become empty and numeric conversions
throw. Supply all parameters explicitly. Values are read once during
initialization; changing a ROS node parameter later does not update `cfg_` or
reconfigure the serial port.

### Linux serial transport

> **Do not** clone `https://github.com/joshnewans/serial.git` package anymore on the robots.

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

For the matching firmware with `DO_RESPOND_OK` enabled, set the hardware
`baud_rate` parameter to **115200**. The empty startup command must return
`* OK\r`; motor and PID commands must return `m OK\r` and `u OK\r`.
Missing or invalid acknowledgments fail activation, deactivation, or motor writes.
Health replies contain voltage in millivolts, current in milliamperes, and free
memory in bytes; sonar replies contain front-right, front-left, back-right, and
back-left distances in centimeters. Invalid telemetry preserves the last valid
state. Seggy currently reports four zero sonar distances. Dragger returns measured
sonar distances (`255` means out of range), measured battery voltage/current, and
free RAM. The driver converts centimeters to meters and millivolts/milliamperes
to volts/amperes. Both pasted firmware versions acknowledge `u` without applying
PID gains: Seggy does nothing, while Dragger parses the values but leaves the gain
assignments commented out. Both use motor setpoints in percent (`-100..100`).

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

### Optional finite sonar no-return distance

By default, sonar values above 250 cm are exported as positive infinity.
To export a finite sentinel instead, add this optional ros2_control hardware
parameter inside the URDF/Xacro `<hardware>` element:

```xml
<param name="sonar_no_return_range">1000.0</param>
```

The value must be finite and greater than 2.5 meters. It applies to all four
sonars; readings at or below 250 cm keep their existing centimeter-to-meter
conversion. Omit the parameter to restore infinity. This is an initialization
parameter and requires restarting the hardware component.

For Nav2 collision monitor, also set `max_range: 1000.0` in each of the four
`range_sensor_broadcaster` controller parameter blocks. Otherwise the sentinel
is outside the declared span and the source can still trigger a stop.
The sentinel represents no return, not a real measurement or an increase in
the physical sensor range. Consumers will see synthetic obstacle points at
that distance; keep it outside collision monitor zones and approach checks.

### Minimal on-robot tracer (Lyrical)

This bench launch runs the real `DiffDriveArduino` plugin, controller manager,
`diff_drive_controller`, joint-state broadcaster, and a small Python tracer.
No navigation stack or external robot description is needed. Stop other nodes
using the serial device before starting it, and raise the wheels for the first test.

Copy `config/tracer_hardware.yaml` and `config/tracer_controllers.yaml` somewhere
editable. Set the serial device, encoder counts, PWM conversion, battery settings,
and measured wheel radius/separation for your robot. The supplied values are
examples, not calibrated robot settings. The controller stops commanding motion
when commands are older than 0.5 seconds; firmware must handle host/serial loss.

```bash
colcon build --packages-select diffdrive_arduino
source install/setup.bash
ros2 launch diffdrive_arduino tracer.launch.py \
  hardware_config:=/absolute/path/to/tracer_hardware.yaml \
  controllers:=/absolute/path/to/tracer_controllers.yaml
```

The tracer prints every received joint-state and odometry message and every
`/cmd_vel` command. It stamps incoming `geometry_msgs/msg/Twist` commands and
forwards them to `/diff_controller/cmd_vel` (`TwistStamped`, as used in Lyrical).
The controller-manager console also prints each completed serial TX/RX transaction
at DEBUG level: `e` contains raw encoder counts; `h` contains millivolts,
milliamperes and free-memory bytes; `p` contains front-right, front-left,
back-right and back-left centimeters. Startup, PID and motor acknowledgments
are included. This traces polled replies, not unsolicited serial traffic;
incomplete exchanges are reported as transport errors. Battery and sonar are
visible in these raw logs without additional sensor broadcasters. Logging every
sample can affect timing, so this launch is intended for diagnosis.

From another sourced terminal, send a small forward command continuously:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05}, angular: {z: 0.0}}'
```

Stop the publisher with Ctrl-C (the controller timeout then stops the commands),
or explicitly send zero:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

Use `linear.x: 0.0` and a small `angular.z` to test turning. Linear velocity is
in m/s and angular velocity in rad/s. Ctrl-C in the launch terminal shuts down
the test. To attach only the tracer to an existing compatible controller setup,
run `ros2 run diffdrive_arduino tracer.py`; it uses the topic names above and
supports ordinary ROS remapping arguments. Avoid another cmd_vel bridge publishing
to the same controller at the same time.

For compact output, add `output_mode:=compact` to the tracer launch command.
This prints the latest raw encoder counts, health (mV, mA, free memory), sonar
(cm), wheel positions/velocities, planar odometry and last received cmd_vel in
about five lines per second. The hardware and Python summaries have independent
one-second timers. Message ages identify old samples; health/sonar age `-1`
means no valid reading yet. A displayed cmd_vel is the last received request,
not proof of an active command after the controller timeout. Serial DEBUG output
is disabled in this mode; warnings and errors remain visible. Intermediate
samples are summarized rather than printed. The default `output_mode:=verbose`
retains the full per-message output. When running the tracer alone, use
`ros2 run diffdrive_arduino tracer.py --ros-args -p output_mode:=compact`;
hardware summaries require the launch's optional `compact_trace` hardware setting.
