#!/usr/bin/env bash

X_PATH="/media/julian/ssdext/xilinx"
X_VERSION="2019.2"
ROS_VERSION="melodic"

echo "-- Test target 'software'"
. "${X_PATH}/Vitis/${X_VERSION}/settings64.sh" && make clean software &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"

sleep 1

echo "-- Test target 'test'"
. "${X_PATH}/Vitis/${X_VERSION}/settings64.sh" && . /opt/xilinx/xrt/setup.sh &> /dev/null && make test HW_TARGET=sw_emu &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"

sleep 1

echo "-- Test ROS test nodes"
. "/opt/ros/${ROS_VERSION}/setup.bash" && cd test && rm -rf build/ devel/ && catkin_make &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"
