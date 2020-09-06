#!/usr/bin/env bash

X_PATH="/media/julian/ssdext/xilinx"
X_VERSION="2019.2"
ROS_VERSION="melodic"

. "${X_PATH}/Vitis/${X_VERSION}/settings64.sh" && . "/opt/ros/${ROS_VERSION}/setup.bash" && . /opt/xilinx/xrt/setup.sh &> /dev/null

echo "-- Test target 'software'"
make clean_software software &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"

sleep 1

echo "-- Test target 'test_sensor_sync'"
make test_sensor_sync HW_TARGET=sw_emu &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"

sleep 1

echo "-- Test target 'test_zmq_client'"
make test_zmq_client HW_TARGET=sw_emu &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"

sleep 1

echo "-- Test target 'test_hdf5'"
make test_hdf5 HW_TARGET=sw_emu &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"

sleep 1



echo "-- Test ROS test nodes"
make clean_ros_nodes ros_test_nodes &> /dev/null

if [ ! $? -eq 0 ]; then echo "FAIL" && exit 1; fi
echo "   done"
