>
> This is original https://github.com/joshnewans/diffdrive_arduino code which I asked ChatGPT to fix at its discretion.
>
> Fixed the stability issues in diffdrive_arduino:
> - Encoder baselines, rollover handling, and discontinuity detection.
> - Period-based velocity calculation.
> - Strict serial parsing, bounded timeouts, and exception handling.
> - Explicit stopping on deactivation; best-effort stop and disconnect on faults.
> - Configuration and command validation.
> - Runnable Lyrical launch examples and updated documentation.
> Validation passed: Lyrical build, 17 regression tests, and a full fake-robot launch verifying movement and stale-command stopping.
> **Physical hardware remains untested**. Parameters and recovery behavior are documented in the README.
>
> The **["non-zeroed encoders" problem](https://github.com/slgrobotics/diffdrive_arduino/tree/jn-lyrical-allfixes#encoder-and-fault-behavior)** hit me specifically while porting my code to Lyrical - the robot started 24 km from odom and map frames. Taking initial encoder values as offset solved it.
-----------------------------------

# diffdrive_arduino

ROS 2 Lyrical ros2_control hardware plugins for a two-wheel differential drive:

- `diffdrive_arduino/DiffDriveArduino` communicates with [ROSArduinoBridge firmware](https://github.com/joshnewans/ros_arduino_bridge).
- `fake_robot/FakeRobot` integrates commanded wheel velocities without hardware.

## Build and test

Install ROS 2 Lyrical and its ros2_control/controller packages. Put this repository
and a ROS 2 version of `serial` in your colcon workspace's `src` directory.
The ROS 2 serial package must export its headers and library through ament.

```bash
source /opt/ros/lyrical/setup.bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to diffdrive_arduino --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select diffdrive_arduino --event-handlers console_direct+
colcon test-result --verbose
```

Linux regression tests use pseudo-terminal serial emulation; they do not access
physical devices. They cover encoder initialization and rollover, malformed
responses, timeouts, invalid configuration/commands, stopping and reactivation.

## Run the example

```bash
ros2 launch diffdrive_arduino fake_robot.launch.py
```

The included minimal URDF uses 0.05 m wheel radii and 0.3 m wheel separation.
Adjust `controllers/robot_controller_example.yaml` and your URDF for your robot.
The launch starts robot_state_publisher, controller_manager,
joint_state_broadcaster and diff_controller. Lyrical receives the robot
description from robot_state_publisher's topic.

Send a stamped velocity command; Lyrical's diff_drive_controller requires
`geometry_msgs/msg/TwistStamped`:

```bash
ros2 topic pub --rate 10 /diff_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: auto, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

For Arduino hardware, set the serial device and encoder resolution:

```bash
ros2 launch diffdrive_arduino test_robot.launch.py \
  device:=/dev/ttyUSB0 enc_counts_per_rev:=1920 firmware_loop_rate:=30
```

You can also use `robot.launch.py use_fake_hardware:=false`. Ensure the firmware
has base control enabled, the serial baud rates agree, and the user has permission
to access the serial device. Opening USB serial often resets an Arduino; the
plugin waits for its startup delay during configuration before sending commands.

## Hardware parameters

Set these inside your `<ros2_control><hardware>` element:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `left_wheel_name`, `right_wheel_name` | Required | Distinct names of the two URDF joints |
| `device` | `/dev/ttyUSB0` | Serial port |
| `baud_rate` | `57600` | Firmware serial baud rate |
| `enc_counts_per_rev` | `1920` | Encoder counts per **wheel** revolution, including gearing and decoding |
| `loop_rate` | `30` | Arduino firmware PID rate in Hz; independent of controller_manager's update rate |
| `timeout` | `10` | Total serial transaction timeout in milliseconds |
| `startup_delay_ms` | `2000` | Boot wait after opening serial, outside the control loop |
| `max_encoder_delta` | `enc_counts_per_rev` | Largest accepted absolute tick change per read after rollover correction |
| `max_motor_command` | `32767` | Largest accepted absolute motor command in ticks per firmware PID frame |
| `pid_p`, `pid_d`, `pid_i`, `pid_o` | `30`, `20`, `0`, `100` | Firmware PID values; `pid_o` must be positive |

Each wheel must expose one `velocity` command interface and both `position` and
`velocity` state interfaces, all of type `double`. The fake plugin only uses
wheel names and encoder resolution.

Choose `timeout` so a read plus a motor write fits within your control period,
with room for computation and scheduling. At 50 Hz that budget is 20 ms; two
10 ms timeouts can consume the entire period on a fault. Transactions are bounded
by a steady-clock deadline and a 128-byte reply limit, but synchronous serial I/O
is not a hard real-time implementation. Slow links may need a lower update rate.

The upstream ROSArduinoBridge firmware expects motor targets in ticks per PID
frame. `loop_rate` must match its `PID_RATE`, normally 30 Hz. PID values must fit
the firmware's signed 16-bit integers and its 15-character combined argument
buffer. Choose a smaller `max_motor_command` appropriate for your drivetrain;
the default is a protocol range, not a physical speed limit.

## Encoder and fault behavior

Configuration and activation sample existing signed 32-bit counters as a baseline.
Wheel positions start at zero and accumulate valid tick differences; they remain
continuous across reactivation. The plugin does not reset the Arduino's counters.
Read velocity uses the period supplied by ros2_control. Signed 32-bit rollover is
handled in either direction.

Malformed, partial, out-of-range or timed-out replies leave positions unchanged.
An implausible tick change, invalid command or serial failure returns ERROR,
clears commands, attempts a zero-speed command and disconnects. Recovery requires
an explicit hardware configuration and activation; communication is not retried
silently. Deactivation sends a zero-speed command and verifies its acknowledgement.
Cleanup and shutdown stop and close the port. Error handling and destruction use
a best-effort stop when acknowledgement is unavailable.

`max_encoder_delta` rejects large counter discontinuities rather than publishing
an odometry jump. Set it above the largest legitimate displacement per sample,
including expected scheduling delays. The protocol cannot distinguish a small
firmware reset from actual motion, or detect every reconnect/reset. A reset while
inactive is baselined on the next activation; motion between the last inactive
sample and activation is not integrated. Firmware with different counter widths
needs a corresponding rollover implementation.

A disconnected cable cannot deliver a stop command. Keep the firmware's motor
watchdog enabled and use an appropriate watchdog interval for your robot. The
controller example also stops stale velocity commands after 0.5 seconds. Physical
hardware behavior still needs validation with your firmware and drivetrain.

Protocol reference: [ROSArduinoBridge firmware](https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_firmware/src/libraries/ROSArduinoBridge/ROSArduinoBridge.ino).
