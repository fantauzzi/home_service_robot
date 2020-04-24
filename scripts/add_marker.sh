#!/bin/bash
PICK_OBJECTS=$(rospack find pick_objects)

PICK_OBJECTS_WORLD=${PICK_OBJECTS}/worlds/training_world.world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PICK_OBJECTS_WORLD}" &
sleep 2

MAP_FILE=${PICK_OBJECTS}/maps/training_world.yaml
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${MAP_FILE} initial_pose_a:=.0" &
sleep 2

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

xterm -e "rosrun add_markers add_markers" &

wait