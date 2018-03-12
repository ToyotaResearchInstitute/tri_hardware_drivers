# lightweight_ur_interface
Lightweight ROS driver for Universal Robots platforms. Exclusively uses the realtime status/command interface and provides velocity commands, pure position control, and path/trajectory execution with built-in trajectory retiming.

## Dependencies

- [ROS Kinetic](http://ros.org)

ROS provides the build system, Catkin, and IPC with the driver. May work with other ROS variants, but has not been tested outside of Ubuntu 16.04.* and ROS Kinetic.

## Optional Dependencies (real or simulated robot hardware)

- Universal Robots UR10, UR5, or UR3 robot

- Universal Robots URSIM simulator

## Build

Clone into an existing Catkin workspace and build with `catkin_make`.

## Functionality

Four ROS nodes are tested and ready to use:

1. `ur_minimal_hardware_interface` - This is the main "driver" that communicates with the robot to publish robot status and send commands to the robot. All command and status topic names are configurable via parameters.

- Robot state is published in the form of `sensor_msgs/JointState` for the joints, `geometry_msgs/PoseStamped` for the end-effector pose, `geometry_msgs/TwistStamped` for the end-effector velocity, and `geometry_msgs/wrenchStamped` for the estimated end-effector forces/torques.

- Commands are accepted in the form of joint velocities or end-effector velocities.

2. `ur_position_controller` - A pure joint position controller. Given a target position for every joint of the robot, commands the arm to move to the target positions using joint velocity commands. It does *not* perform any sort of smooth velocity parametrization other that applying velocity and acceleration limits. This controller is mean for small motions that can be easily modelled mathematically. Large motions should be performed using the `ur_trajectory_controller`.

3. `ur_cartesian_controller` - A pure cartesian pose controller. Given a target pose of the end-effector, commands the robot to move towards the target pose using cartesian velocity commands. It does *not* perform any sort of smooth velocity parametrization other that applying velocity and acceleration limits. Note that neither this controller nor the UR robot controller check if the target pose can actually be reached. Cartesian pose controllers are meant for small motions in "known good" areas of the workspace. If you need precise guarantees of robot motions, you should generate motion separately with a planner and IK solver and use the joint trajectory or joint position controllers to execute it.

4. `ur_trajectory_controller` - A joint path/trajectory controller. Given a joint path or trajectory, it computes a time-optimal reparametrization of the path using the TOTP algorithm using the active joint velocity and acceleration limits. The reparametrized path is then executed using joint velocity control. This is the controller to use for large arm motions, or if you have a full path or trajectory produced by a motion planner.

The three controllers only send commands to the robot when they are active, so multiple controllers can be available simultaneously so long as only one is active at a given time. When none of the controllers is active, the robot can enter teach mode from the pendant (unlike the ROS control driver for `ur_modern_driver`).

## Development

1. Improve the flexibility of the configuration system and allow for runtime reconfiguration of joint limits and controller gains.

2. Improve the cartesian velocity command interface to accept commands either in the base frame of the robot, or in the end-effector frame (and then convert to base-frame to send to the robot). Likewise, publish the end-effector velocity on two separate topics, one for each frame (end-effector twist would be prefered for many reasons!).

3. Ongoing development work to enable a cartesian impedance control mode similar to that available on the Kuka IIWA robots. Outstanding issues:

- The current realtime interface does not allow commands to be run sequentially, so enabling `force_mode` via command is not enough.

- The robot cannot be commanded purely by realtime `force_mode` commands.

The planned solution is a second hardware interface node that reads state over the existing realtime interface, but sends a larger control script to the controller at startup and then sends commands directly to the script. This also allows teach/freedrive mode to be enabled from ROS, which allows more user-friendly teaching.

## Run

1. *Prerequisite*: Your computer must be connected to a simulated or real Universal Robots controller. Note the controller IP address, since you'll need it to start the driver.

2. *Prerequisite*: ROS must be running. You can start directly by running `roscore` in a terminal, or by running a `roslaunch` file.

3. Start the hardware interface. As an example, using the default topic names (you should not change them unless you need to) and scaling the velocity and acceleration limits to 50% of what the hardware supports:

```
~$ rosrun lightweight_ur_interface ur_minimal_hardware_interface _robot_hostname:="<YOUR ROBOT's IP ADDRESS>" _velocity_limit_scaling:=0.5 _acceleration_limit_scaling:=0.5
```

4. Start one or more more controllers. These controllers must use equivalent or lower velocity and acceleration scaling as the hardware interface, or unexpected motion may result! You can also provide the control gains to use for the PD controllers in the form of kp and kd. Examples for each of the three controllers:

```
~$ rosrun lightweight_ur_interface ur_position_controller _velocity_limit_scaling:=0.5 _acceleration_limit_scaling:=0.5 _base_kp:=1.0 _base_kd:=0.1

```
```
~$ rosrun lightweight_ur_interface ur_cartesian_controller _max_linear_velocity:=0.5 _max_angular_velocity:=1.0 _translation_kp:=1.0 _translation_kd:=0.1 _rotation_kp:=1.0 _rotation_kd:=0.1
```
```
~$ rosrun lightweight_ur_interface ur_trajectory_controller _velocity_limit_scaling:=0.5 _acceleration_limit_scaling:=0.5 _base_kp:=1.0 _base_kd:=0.1
```
