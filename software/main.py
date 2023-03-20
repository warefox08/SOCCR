#!/usr/bin/env python
from lzma import MF_BT3
import sys, os
import rospy
sys.path.append(os.path.join(os.getcwd(), "motion")) #add function folder to path
sys.path.append(os.path.join(os.getcwd(), "laser_tracking")) #add function folder to path
sys.path.append(os.path.join(os.getcwd(), "comms"))
sys.path.append(os.path.join(os.getcwd(), "laser_tracking/auto_nav/scripts"))

#print(sys.path)
#import motion_functions as mf
import ltf_class as lt
import listener_class
import goal_pose2 as gs

def main():
	rospy.init_node('python_node')
	# debug=0
	# if debug==1:
	# 	[test1, test2] = mf.import_test
	# 	print(test1)
	# 	print(test2)
	
#	velocity_publisher, vel_msg = mf.init()

	listener = listener_class.listener()
	listener.init()

	tracker = lt.laser_tracker()

	print("START")
	while(1):
		if listener.flag == 1:
			print("flag raised")
			# angle_deg = 90
			# distance = 0.5
			#angle_deg = listener.angle
			#distance = listener.distance/1000
			if (tracker.color_flag and tracker.depth_flag):
				angle_deg, distance_z, distance_x = tracker.find_vector_to_laser()
				print ("\nd_z: ", distance_z)
				print ("d_x: ", distance_x)
				#mf.send_motion_command(vel_msg, velocity_publisher, angle_deg, distance_z/1000)
				gs.publishMoveBaseGoalWaitForReply(distance_x, distance_z)
#				gs.send_command(navclient, goal, distance_z, distance_x)
			else:
				print("no laser")
			listener.lower_flag()
			print("flag_lowered")
#		else:
#			print("no flag")
			

if __name__ == "__main__":
	main()
