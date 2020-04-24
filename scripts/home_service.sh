#!/bin/bash
echo "Hit Ctrl+C to stop and close everything..."
PICK_OBJECTS=$(rospack find pick_objects)

PICK_OBJECTS_WORLD=${PICK_OBJECTS}/worlds/training_world.world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PICK_OBJECTS_WORLD}; bash" &
sleep 5

MAP_FILE=${PICK_OBJECTS}/maps/training_world.yaml
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${MAP_FILE} initial_pose_a:=.0; bash" &
sleep 2

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch; bash" &

xterm -e "rosrun add_markers add_markers; bash" &
sleep 2

xterm -e "rosrun pick_objects pick_objects; bash" &

wait
