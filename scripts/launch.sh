#!/bin/bash

script_dir="$(dirname "$0")"
. ${script_dir}/_ros_helper.sh #include some common functions

echo "dir = $ws_dir"

#my_robot_dir=$ws_dir/src/my_robot
#world_file=$my_robot_dir/worlds/homeservice.world
#map_file=$my_robot_dir/maps/homeservice.yaml

echo "world_file = $world_file"
echo "map_file = $map_file"


start_xterm gazebo
sleep 5
start_xterm roscore
sleep 5
start_xterm " rosrun rviz rviz"
