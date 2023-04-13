#!/usr/bin/env python
from lzma import MF_BT3
import sys, os
import rospy

#sys.path.append(os.path.join(os.getcwd(), "motion")) #add function folder to path
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
		print("flag raised")
		if listener.command == "M":
			if (tracker.color_flag and tracker.depth_flag):
				angle_deg, distance_z, distance_x = tracker.find_vector_to_laser()
				print ("\nd_z: ", distance_z)
				print ("d_x: ", distance_x)
				#mf.send_motion_command(vel_msg, velocity_publisher, angle_deg, distance_z/1000)
				gs.move_command(distance_x, distance_z, 0)
			else:
				print("no laser")
		if listener.command == "R":
			gs.move_command(0, 0, -0.523)
		if listener.command == "L":
			gs.move_command(0,0, 0.523)

		listener.lower_flag()
		print("flag_lowered")

	#		else:
	#			print("no flag")