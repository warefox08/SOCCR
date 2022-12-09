#!/usr/bin/env python
from lzma import MF_BT3
import sys, os
sys.path.append(os.path.join(os.getcwd(), "motion")) #add function folder to path
# sys.path.append(os.path.join(os.getcwd(), "laser_tracking")) #add function folder to path
sys.path.append(os.path.join(os.getcwd(), "comms"))

print(sys.path)
import motion_functions as mf
# import laser_tracking_functions as lt

import listener_class

def main():
	# debug=0
	# if debug==1:
	# 	[test1, test2] = mf.import_test
	# 	print(test1)
	# 	print(test2)
	
	velocity_publisher, vel_msg = mf.init()
	# [dc, fov_h, res_h, res_v] = lt.init()
	listener = listener_class.listener()
	listener.init()
	print("START")
	while(1):
		if listener.flag == 1:
			print("flag raised")
			listener.lower_flag()
			# angle_deg, distance = lt.find_vector_to_laser(dc, fov_h, res_h, res_v)
			angle_deg = 10
			distance = 0.5
			mf.send_motion_command(vel_msg, velocity_publisher, angle_deg, distance)
			

if __name__ == "__main__":
	main()
