#!/bin/bash
echo "Hit Ctrl+C to stop and close everything..."
PICK_OBJECTS=$(rospack find pick_objects)

PICK_OBJECTS_WORLD=${PICK_OBJECTS}/worlds/training_world.world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PICK_OBJECTS_WORLD}; bash" &
sleep 2

xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch; bash" &
sleep 2

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch; bash" &
sleep 2

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch; bash" &

wait
