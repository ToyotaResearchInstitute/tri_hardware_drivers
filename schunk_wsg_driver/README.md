# schunk_wsg_driver
ROS driver for Schunk WSG grippers supporting UDP and CAN control interfaces

## Dependencies

- Linux kernel with socketcan support enabled (enabled by default in Ubuntu 16.04.* and others)

CAN is supported using the socketcan system in Linux that allows CAN bus communication in a manner similar to network sockets.

- [ROS Kinetic](http://ros.org)

ROS provides the build system, Catkin, and IPC with the driver.


## Optional Dependencies

- CAN adapter supporting socketcan

To use the CAN interface, your CAN adapter must support socketcan. A full list of supported drivers is here [Embedded Linux Wiki: CAN Bus](https://elinux.org/CAN_Bus)

## Build

Clone into an existing Catkin workspace and build with `catkin_make`.

## Run

Common parameters:

- `interface_type` Selects which interface type to use. Valid options are `udp` or `can`.

- `control_rate` Sets the rate at which the driver loops, publishing status and forwarding comands. Valid options are positive, non-zero.

- `command_topic` Sets the ROS topic name used to receive command messages

- `status_topic` Sets the ROS topic name used to publish status messages

### UDP interface

1. *Prerequisite*: Using the gripper configuration webpage, configure the control interface to UDP and set the IP address appropriately.

2. Parameters:

- `interface_type` must be set to `udp`

- `gripper_ip_address` must be set

- `gripper_port` must be set if your gripper is configured to use a port other than the default 1500

- `local_port` must be set if your gripper is configured to use a port other than the default 1501

3. Start driver node. You can integrate the parameters and starting into a ROS launch file, or you can provide all parameters on the command line:

```
~$ rosrun schunk_wsg_driver schunk_wsg_driver_node _interface_type:="udp" _gripper_ip_address:="192.168.1.2"
```

### CAN interface

1. *Prerequisite*: Using the gripper configuration webpage, configure the control interface to CAN, set the CAN bitrate (use the highest speed possible!) and set the CAN id appropriately.

2. *Prerequisite*: Configure and bring up the CAN interface on your computer. See [Embedded Linux Wiki: Bringing CAN interface up](http://elinux.org/Bringing_CAN_interface_up) for more. For example, to set interface `can0` to run a 1 Mbit/s and bring it up, you would use:

```
~$ sudo ip link set can0 type can bitrate 1000000
~$ sudo ip link set up can0

```

3. Parameters:

- `interface_type` must be set to `can`

- `socketcan_interface` must be set

- `gripper_base_can_id` must be set

4. Start driver node. You can integrate the parameters and starting into a ROS launch file, or you can provide all parameters on the command line:

```
~$ rosrun schunk_wsg_driver schunk_wsg_driver_node _interface_type:="can" _socketcan_interface:="can0" _gripper_base_can_id:="100"
```
