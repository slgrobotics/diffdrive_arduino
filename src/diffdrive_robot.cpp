#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "diffdrive_arduino/diffdrive_arduino.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diffdrive_robot");
  ros::NodeHandle n("~");

  DiffDriveArduino::Config robot_cfg;

  // Attempt to retrieve parameters. If they don't exist, the default values from the struct will be used
  n.getParam("robot_loop_rate", robot_cfg.loop_rate); // Hz
  

  DiffDriveArduino robot(robot_cfg);
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prevTime = ros::Time::now();

  ros::Rate loop_rate(robot_cfg.loop_rate);  // ROS Rate Hz

  while (ros::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();

    loop_rate.sleep();
  }
}
