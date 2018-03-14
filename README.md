# tri_hardware_drivers
Single repo for ROS drivers for third-party robotics hardware used at TRI

## Dependencies

- [ROS Kinetic](http://ros.org)

ROS provides the build system, Catkin, and IPC with the driver. May work with other ROS variants, but has not been tested outside of Ubuntu 16.04.* and ROS Kinetic.

- [control_msgs](http://wiki.ros.org/control_msgs)

ROS messages for robot control. Depending on your ROS installation type (-base, -desktop, -desktop-full), you may need to install these separately.

- [tf2_msgs](http://wiki.ros.org/tf2_msgs)

ROS messages for the TF system. Depending on your ROS installation type (-base, -desktop, -desktop-full), you may need to install these separately.

- [libmodbus](http://libmodbus.org)

libmodbus provides support for Modbus RTU and Modbus TCP communications.

- Linux kernel with socketcan support enabled (enabled by default in Ubuntu 16.04.* and others)

CAN is supported using the socketcan system in Linux that allows CAN bus communication in a manner similar to network sockets.
