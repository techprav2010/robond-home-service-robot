#!/bin/bash

function get_ws_dir()
{
    SCRIPT_DIR=`dirname $0`
    cd $SCRIPT_DIR/..
    echo `pwd`
}

ws_dir="`get_ws_dir`"
cd $ws_dir

#my_robot_dir=$ws_dir/src/my_robot
#world_file=$my_robot_dir/worlds/home_service.world
#map_file=$my_robot_dir/maps/home_service.yaml

my_robot_dir=$ws_dir/src/my_robot
world_file=$my_robot_dir/worlds/homeservice.world
map_file=$my_robot_dir/maps/homeservice.yaml

#echo "BASH_SOURCE ${BASH_SOURCE[0]}"

function roslaunch_xterm()
{
 action="roslaunch ${1} ${2} ${3} ${4} ${5} "
 echo $action
 xterm  -e  "cd $ws_dir && source devel/setup.sh && $action" &
 #sleep 5
}

function rosrun_xterm()
{
 action="rosrun ${1} ${2} ${3} ${4} ${5}  ${6}  ${7} "
 echo $action
 xterm  -e  "cd $ws_dir && source devel/setup.sh && $action" &
 #sleep 5
 echo "done $action "
}

function start_xterm()
{
 action="${1} ${2} ${3} ${4} ${5}  ${6}  ${7} "
 echo $action
 xterm  -e  "cd $ws_dir && source devel/setup.sh && $action" &
 #sleep 5
}



