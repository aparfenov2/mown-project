#!/usr/bin/env bash
echo 1 > /proc/sys/vm/drop_caches
ip4=$(/sbin/ip -o -4 addr list wlan0 | awk '{print $4}' | cut -d/ -f1)
export ROS_IP=$ip4
export ROS_IP
roslaunch realsense2_camera opensource_tracking.launch
