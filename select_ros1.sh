#!/bin/bash
#
# Sets symlinks for development with ROS 1.

set -euo pipefail

function make_symlinks()
{
  cd $1

  if test -f CMakeLists.txt
  then
    echo "$1/CMakeLists.txt already exists"
  else
    ln -sT CMakeLists.txt.ros1 CMakeLists.txt
  fi

  if test -f package.xml
  then
    echo "$1/package.xml already exists"
  else
    ln -sT package.xml.ros1 package.xml
  fi

  cd $OLDPWD
}

make_symlinks ati_netcanoem_ft_driver
make_symlinks dji_robomaster_ep_driver
make_symlinks lightweight_ur_interface
make_symlinks robotiq_ft_driver
make_symlinks robotiq_2_finger_gripper_driver
make_symlinks robotiq_3_finger_gripper_driver
make_symlinks schunk_wsg_driver
make_symlinks tri_hardware_drivers
make_symlinks tri_mocap_common
