## diffdrive_arduino - a unified ROS2 *base* driver 

Implements *diffdrive_arduino* package ([ros2_control](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts) interfaces)
which connects to an Arduino Mega 2560, Teensy 4.x or RPi Pico.

Uses simple strings-based protocol to communicate to microcontroller, which controls wheels motors and encoders, monitors battery and may control some sensors (e.g. sonars).

For matching Arduino IDE code see:
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Pico
- https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Seggy  (BLDC motor wheels)

Please refer to [my Project Wiki](https://github.com/slgrobotics/articubot_one/wiki) for detailed instructions.

**Credits:** Original code by Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros
