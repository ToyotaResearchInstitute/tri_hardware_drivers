# tri_hardware_drivers
Single repo for ROS drivers for third-party robotics hardware used at TRI.

Most drivers support being built with ROS 1 (Kinetic+) _or_ ROS 2 (Galactic+),
with the desired ROS version being selected by symlinks to `CMakeLists.txt` and
`package.xml` for each package.

To select ROS 1 for _all_ driver packages, run `$ ./select_ros1.sh`.

To select ROS 2 for _all_ driver packages, run `$ ./select_ros2.sh`.

## Dependencies

- [ROS 1 Kinetic+ or ROS 2 Galactic+](http://ros.org)

ROS provides the build system and IPC with the drivers.

- [control_msgs](http://wiki.ros.org/control_msgs)

ROS messages for robot control. Depending on your ROS installation type (-base, -desktop, -desktop-full), you may need to install these separately.

- [tf2_msgs](http://wiki.ros.org/tf2_msgs)

ROS messages for the TF system. Depending on your ROS installation type (-base, -desktop, -desktop-full), you may need to install these separately.

- [common_robotics_utilities](https://github.com/ToyotaResearchInstitute/common_robotics_utilities)

Common functions for math operations, trajectory parametrization, data de/serialization, and logging.

- [libmodbus](http://libmodbus.org)

libmodbus provides support for Modbus RTU and Modbus TCP communications.

- Linux kernel with socketcan support enabled (enabled by default in Ubuntu 16.04.* and others)

CAN is supported using the socketcan system in Linux that allows CAN bus communication in a manner similar to network sockets.
