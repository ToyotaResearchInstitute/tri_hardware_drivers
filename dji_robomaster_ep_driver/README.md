# dji_robomaster_ep_driver
ROS driver for DJI's Robomaster EP robot, connected via USB or WiFi

## Dependencies

- [ROS Kinetic+](http://ros.org)

ROS provides the build system, Catkin, and IPC with the driver.

## Build

Clone into an existing Catkin workspace and build with `catkin_make`.

## Run

1. *Prerequisite*: Activate your Robomaster EP robot. See documentation provided with the robot.

Connections using USB and WiFI are supported, although USB is more reliable and safer. Ensure that the USB cable connecting robot to control computer is secure and cannot be caught and pulled loose, as the robot's onboard controller continues to execute motion commands for some time after a connection is lost.

2. Driver node

Parameters:

- `robot_ip_address` defaults to `192.168.42.2`, as used when connecting via USB

- `robot_port` defaults to `40923`, as used when connecting via USB

- `robot_name` used to prefix topics for command (`<robot_name>/cmd_vel`) and status (`<robot_name>/odometry`, `<robot_name>/battery_percent`), defaults to `robomaster_ep`

- `odometry_frame_name` TF frame name for odometry, defaults to `world`

- `robot_frame_name` TF frame name for robot, defaults to `robomaster_body`, velocity commands must be in this frame

- `loop_hz` frequency for driver spin loop, defaults to 60 Hz

To run the driver node:

```
~$ rosrun dji_robomaster_ep_driver dji_robomaster_ep_driver_node _robot_name:="robomaster_ep" _odometry_frame_name:="world" _robot_frame_name:="robomaster_body"
```

3. Joystick teleoperation node

*Prerequisite*: Connect joystick, start ROS driver (ex. `~$ rosrun joy joy_node`)

Parameters:

- `joystick_type` selects between supported joystick mappings, Xbox One (`xbox_one`) and Logitech Extreme 3D Pro (`3d_pro`) supported, defaults to `xbox_one`

- `joystick_topic` topic for messages from joystick, defaults to `joy`

- `command_topic` topic for velocity commands, defaults to `robomaster_ep/cmd_vel`

- `command_frame` TF frame name for velocity commands, defaults to `robomaster_body`

- `max_linear_velocity` maximum linear velocity, defaults to 3.5 m/s, a lower limit is *strongly recommended*

- `max_angular_velocity` maximum angular velocity, defaults to 10.0 rad/s, a lower limit is *strongly recommended*

- `loop_hz` frequency for publishing velocity commands, defaults to 30 Hz

To run the joystick teleoperation node:

```
~$ rosrun dji_robomaster_ep_driver joystick_teleop_node _joystick_type:="xbox_one" _joystick_topic:="joy" _max_linear_velocity:="1.0" _max_angular_velocity:="1.0"
```
