#!/usr/bin/env python3
from lzma import MF_BT3
import sys, os
import rospy
import math

sys.path.append(os.path.join(os.getcwd(), "motion")) #add function folder to path
#sys.path.append(os.path.join(os.getcwd(), "laser_tracking")) #add function folder to path
#sys.path.append(os.path.join(os.getcwd(), "comms"))
#sys.path.append(os.path.join(os.getcwd(), "laser_tracking/auto_nav/scripts"))

#import motion_functions as mf
import ltf_class as lt
import listener_class
import goal_pose_copy_2 as gs

rospy.init_node('python_node')

#	velocity_publisher, vel_msg = mf.init()

listener = listener_class.listener()
listener.init()

tracker = lt.laser_tracker()

print("START")
while(1):
	if listener.flag == 1:
#		print("flag raised")
		if listener.command == "D":
			print("move 1m forward")
			gs.move_command(1, 0, 0, 1)
		if listener.command == "M":
#			print (tracker.color_flag)
#			print (tracker.depth_flag)
			rospy.set_param('yaw_goal_tolerance', 6.28)
			rospy.set_param('xy_goal_tolerance', 0.1)
			laser_found = 0
			print(tracker.frame_counter)
			if (tracker.color_flag and tracker.depth_flag):
				laser_found, _, distance_x, distance_z = tracker.search_for_laser()
			if laser_found:
				listener.pub_feedback("F") #found laser
				print ("\nd_z: ", distance_z/1000)
				print ("d_x: ", distance_x/1000)
				alpha = math.tan(distance_z/distance_x)
				o_z = math.sin(alpha/2)
				o_w = math.cos(alpha/2)
				gs.move_command(distance_x/1000, distance_z/1000, 0, 1)
#				gs.move_command(distance_x/1000, distance_z/1000, o_z,o_w)
#				gs.move_command(1, -0.5, 0, 0)
				print("move sent")
			else:
				listener.pub_feedback("N") #no laser
				print("no move sent")
		if listener.command == "R":
		#	gs.move_command(0, 0, -0.259, 0.966) #30 deg
			rospy.set_param('yaw_goal_tolerance', 0.1)
			rospy.set_param('xy_goal_tolerance', 10)
			gs.move_command(0, 0, -0.383, 0.924) #45 deg
		if listener.command == "L":
#			gs.move_command(0,0, 0.259, 0.966) #30 deg
			rospy.set_param('yaw_goal_tolerance', 0.1)
			rospy.set_param('xy_goal_tolerance', 10)
			gs.move_command(0, 0, 0.383, 0.924)
		listener.lower_flag()
#		print("flag_lowered")
		listener.pub_feedback("C") #completed motion
		print("C")
	#		else:
	#			print("no flag")
