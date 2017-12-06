#!/bin/bash

#script to run to get everything ready to run with ros
source /opt/ros/r2b3/local_setup.bash
source /opt/ros/r2b3/setup.bash
source ../../../../install/local_setup.bash
source ../../../../install/setup.bash 

#script to run to build the l6470_msg/srvs in javascript (ready to use)
node generate_ros_l6470_interfaces.js
