# ati_netcanoem_ft_driver
ROS driver for ATI Force/Torque sensors using the NETCANOEM interface board

## Dependencies

- Linux kernel with socketcan support enabled (enabled by default in Ubuntu 16.04.* and others)

CAN is supported using the socketcan system in Linux that allows CAN bus communication in a manner similar to network sockets.

- CAN adapter supporting socketcan

To use the CAN interface, your CAN adapter must support socketcan. A full list of supported drivers is here [Embedded Linux Wiki: CAN Bus](https://elinux.org/CAN_Bus)

- [ROS](http://ros.org)

ROS provides the build system and IPC with the driver.

## Setup

Clone into an existing ROS workspace.

This package supports [ROS 1 Kinetic+](http://wiki.ros.org/ROS/Installation) and [ROS 2 Dashing+](https://index.ros.org/doc/ros2/Installation/) distributions.
Make sure to symlink the corresponding `CMakeLists.txt` and `package.xml` files for the ROS distribution of choice:

*For ROS 1 Kinetic+*
```sh
cd ~/ws/src/ati_netcanoem_ft_driver
ln -sT CMakeLists.txt.ros1 CMakeLists.txt
ln -sT package.xml.ros1 package.xml
```

*For ROS 2 Foxy+*
```sh
cd ~/ws/src/ati_netcanoem_ft_driver
ln -sT CMakeLists.txt.ros2 CMakeLists.txt
ln -sT package.xml.ros2 package.xml
```

Use [`rosdep`](https://docs.ros.org/independent/api/rosdep/html/) to ensure all dependencies in the `package.xml` are satisfied:

```sh
cd ~/ws
rosdep install -i -y --from-path src
```

## Build

Use [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) or [`colcon`](https://colcon.readthedocs.io/en/released/) accordingly.

*In ROS 1 Kinetic+*
```sh
cd ~/ws
catkin_make  # the entire workspace
catkin_make --pkg ati_netcanoem_ft_driver  # the package only
```

*In ROS 2 Foxy +*
```sh
cd ~/ws
colcon build  # the entire workspace
colcon build --packages-select ati_netcanoem_ft_driver  # the package only
```

## Run

1. *Prerequisite*: Configure the CAN base address and set the CAN bitrate (use the highest speed possible!). A helper program is provided to do so:

*In ROS 1 Kinetic+*

```
~$ rosrun ati_netcanoem_ft_driver configure_ati_can_ft_sensor <socketcan interface name> <current sensor base can id> <new sensor base can id> <new sensor can baud rate divisor>
```

*In ROS 2 Foxy+*

```
~$ ros2 run ati_netcanoem_ft_driver configure_ati_can_ft_sensor <socketcan interface name> <current sensor base can id> <new sensor base can id> <new sensor can baud rate divisor>
```

See the NETCANOEM interface board documentation for more information.

2. *Prerequisite*: Configure and bring up the CAN interface on your computer. See [Embedded Linux Wiki: Bringing CAN interface up](http://elinux.org/Bringing_CAN_interface_up) for more. For example, to set interface `can0` to run a 1 Mbit/s and bring it up, you would use:

```
~$ sudo ip link set can0 type can bitrate 1000000
~$ sudo ip link set up can0

```

3. Parameters:

- `socketcan_interface` must be set

- `sensor_base_can_id` must be set

- `poll_rate` must be set to a positive non-zero value

- `status_topic` must be set; sets the ROS topic name for publishing force/torque data

- `sensor_frame` must be set; sets the name for the ROS TF frame in which the force/torque measurements are made

- `sensor_calibration_index` must be set; sets the index of the sensor calibration to use

4. Start driver node. You can integrate the parameters and starting into a ROS launch file, or you can provide all parameters on the command line:

*In ROS 1 Kinetic+*
```
~$ rosrun ati_netcanoem_ft_driver ati_netcanoem_ft_driver_node _socketcan_interface:="can0" _sensor_base_can_id:=10 _poll_rate:=100.0 _status_topic:="ati_ft" _sensor_frame:="ati_ft_frame" _sensor_calibration_index:=0
```

*In ROS 2 Foxy+*
```
~$ ros2 run ati_netcanoem_ft_driver ati_netcanoem_ft_driver_node --ros-args -p socketcan_interface:="can0" -p sensor_base_can_id:=10 -p poll_rate:=100.0 -p status_topic:="ati_ft"  -p sensor_frame:="ati_ft_frame" -p sensor_calibration_index:=0
```
