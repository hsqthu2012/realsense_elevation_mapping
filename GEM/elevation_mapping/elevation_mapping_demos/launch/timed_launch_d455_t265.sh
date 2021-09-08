#!/bin/bash
roslaunch elevation_mapping_demos realsense_d455_t265_start.launch &
sleep 10
echo "realsense starting finished!"

roslaunch elevation_mapping_demos realsense_d455_t265_elevation_mapping.launch &
sleep 0.1
wait
exit 0