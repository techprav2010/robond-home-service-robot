#!/bin/bash

script_dir="$(dirname "$0")"
. ${script_dir}/_ros_helper.sh #include some common functions

echo "dir = $ws_dir"

#my_robot_dir=$ws_dir/src/my_robot
#world_file=$my_robot_dir/worlds/homeservice.world
#map_file=$my_robot_dir/maps/homeservice.yaml

echo "world_file = $world_file"
echo "map_file = $map_file"


#1
#Launch turtlebot in the custom world
#export TURTLEBOT_GAZEBO_WORLD_FILE=$world_file
roslaunch_xterm  turtlebot_gazebo turtlebot_world.launch  world_file:=$world_file
sleep 5

#2
#Launch amcl demo
roslaunch_xterm  turtlebot_gazebo amcl_demo.launch map_file:=$map_file
sleep 2

#3
#Launch rviz
roslaunch_xterm  turtlebot_rviz_launchers view_navigation.launch
sleep 10


#4
#run wall_follower
rosrun_xterm  wall_follower wall_follower
sleep 5


