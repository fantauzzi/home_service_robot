#!/bin/bash
PICK_OBJECTS=$(rospack find pick_objects)

PICK_OBJECTS_WORLD=${PICK_OBJECTS}/worlds/training_world.world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PICK_OBJECTS_WORLD}" &
sleep 2

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 2

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 2

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

wait