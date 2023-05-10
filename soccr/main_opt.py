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
        if listener.command == "M": #If command is Move
            #rospy.set_param('yaw_goal_tolerance', 6.28) #Rotate tolerance is 2pi rad (don't care about rotation)
            #rospy.set_param('xy_goal_tolerance', 0.1) #XY tolerance is 0.1m
            client.update_configuration({"xy_goal_tolerance": 0.5, "yaw_goal_tolerance": 3.14159})
            laser_found = 0
            if (tracker.color_flag and tracker.depth_flag): #If both frames have already arrived
                laser_found, _, distance_x, distance_y = tracker.search_for_laser() #Find x and z distace
            if laser_found:
                listener.pub_feedback("F")  #Publish feedback to device that laser has been found 
                gs.move_command(distance_x, distance_y, 0, 1) #Send move command
            else:
                listener.pub_feedback("N")  #Publish feedback to device that no laser found
        if listener.command == "R": #If command is Right turn
            #rospy.set_param('yaw_goal_tolerance', 0.1) #Rotate tolerance is 0.1 rad
            #rospy.set_param('xy_goal_tolerance', 10) #XY tolerance is 10m (don't care about xy pos)
            client.update_configuration({"xy_goal_tolerance": 10.0, "yaw_goal_tolerance": 0.5})
            gs.move_command(0, 0, -0.383, 0.924)  # 45 deg rotation in quaternions
        if listener.command == "L": #If command is Left turn
            # rospy.set_param('yaw_goal_tolerance', 0.1)
            # rospy.set_param('xy_goal_tolerance', 10)
            client.update_configuration({"xy_goal_tolerance": 10.0, "yaw_goal_tolerance": 0.5})
            gs.move_command(0, 0, 0.383, 0.924) # -45 deg
        listener.lower_flag() #Lower listener flag to signify execution is over
        listener.pub_feedback("C")  #Publish feedback to device that execution is over 