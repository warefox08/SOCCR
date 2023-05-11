#!/usr/bin/env python3
#Import necessary libraries
import sys
import os
import rospy
from dynamic_reconfigure.client import Client

import ltf_class as lt
import listener_class
import goal_pose_copy_2 as gs

rospy.init_node('python_node') #Initialise ROS node
client = Client("/move_base/DWAPlannerROS")
# Create and initialise listener and tracker objects
listener = listener_class.listener()
listener.init()

tracker = lt.laser_tracker()

print("START")
while True: #Loop infinitely
    if listener.flag == 1: #Waiting for flag to get raised when any command is sent

        if listener.command == "I":
            tracker.save_image()
            listener.pub_feedback("i")

        if listener.command == "M": #If command is Move
            client.update_configuration({"xy_goal_tolerance": 0.05, "yaw_goal_tolerance": 3.14159})
            laser_found = 0
            if (tracker.color_flag and tracker.depth_flag): #If both frames have already arrived
                laser_found, _, distance_x, distance_y = tracker.search_for_laser() #Find x and z distace
            if laser_found:
                listener.pub_feedback("f")  #Publish feedback to device that laser has been found 
                listener.pub_feedback("X"+str(distance_x/1000))  #Publish x distance to device 
                listener.pub_feedback("Y"+str(distance_y/1000))  #Publish y distance to device 
                gs.move_command(distance_x/1000, distance_y/1000, 0, 1) #Send move command
                listener.pub_feedback("complete")  #Publish feedback to device that execution is over 
            else:
                listener.pub_feedback("n")  #Publish feedback to device that no laser found

        if listener.command == "R": #If command is Right turn
            client.update_configuration({"xy_goal_tolerance": 10.0, "yaw_goal_tolerance": 0.5})
            gs.move_command(0, 0, -0.5, 0.866)  # 60 deg rotation in quaternions
            listener.pub_feedback("r")  #Publish feedback to device that execution is over 

        if listener.command == "L": #If command is Left turn
            client.update_configuration({"xy_goal_tolerance": 10.0, "yaw_goal_tolerance": 0.5})
            gs.move_command(0, 0, 0.5, 0.866) # -60 deg
            listener.pub_feedback("l")  #Publish feedback to device that execution is over 

        listener.lower_flag() #Lower listener flag to signify execution is over
