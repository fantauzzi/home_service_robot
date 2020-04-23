#!/bin/bash
# xterm +hold -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
xterm +hold -e "roslaunch pick_objects world.launch" &
sleep 2
xterm +hold -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 2
xterm +hold -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 2
xterm +hold -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &