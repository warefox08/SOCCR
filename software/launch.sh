#!/bin/bash

echo "BEGIN LAUNCHING"
echo "-----------------"
echo "Bringing up turtlebot"
roslaunch turtlebot3_bringup turtlebot3_robot.launch &
echo "Bring up complete"
echo "Launching camera"
roslaunch realsense2_camera rs_camera.launch camera:=camera1 depth_width:=640 color_width:=640 depth_height:=480 color_height:=480 depth_fps:=6 color_fps:=6 pointcloud_texture_stream:=RS2_STREAM_COLOR enable_sync:=true align_depth:=true initial_reset:=false filters:=pointcloud &
echo "Camera launched"
echo "Starting auto_nav"
roslaunch auto_nav turtlebot3_auto_nav.launch open_rviz:=false &
echo "auto_nav started"
echo "Running rosserial"
rosrun rosserial_python serial_node.py tcp &
echo "Rosserial ready"
wait
echo "-----------------"
echo "FINISHED LAUNCHING"