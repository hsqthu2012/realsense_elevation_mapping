#!/bin/bash
roslaunch elevation_mapping_demos realsense_start.launch &
sleep 3
echo "realsense starting finished!"

roslaunch elevation_mapping_demos realsense_visual_odometry.launch &
sleep 7
echo "realsense visual odometry finished!"

roslaunch elevation_mapping_demos realsense_elevation_mapping.launch &
sleep 0.1
wait
exit 0