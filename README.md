#Team 10
#laser tracking system

#How to Launch

#if launch on laptop
roslaunch turtlebot3_bringup turtlebot3_robot.launch
#if not 
roscore

#start
roslaunch realsense2_camera rs_camera.launch 

roslaunch auto_nav turtlebot3_auto_nav.launch 

rosrun rosserial_python serial_node.py tcp

