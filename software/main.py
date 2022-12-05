#!/usr/bin/env python
from lzma import MF_BT3
import sys, os
# sys.path.append(os.path.join(os.getcwd(), "motion")) #add function folder to path
# sys.path.append(os.path.join(os.getcwd(), "laser_tracking")) #add function folder to path
sys.path.append(os.path.join(os.getcwd(), "comms"))
# import motion_functions as mf
# import laser_tracking_functions as lt
import paho.mqtt.subscribe as subscribe

def main():
	# debug=0
	# if debug==1:
	# 	[test1, test2] = mf.import_test
	# 	print(test1)
	# 	print(test2)
	
	# velocity_publisher, vel_msg = mf.init()
	# [dc, fov_h, res_h, res_v] = lt.init()
	# client = listener.init()
	print("START")
	while(1):
		msg = subscribe.simple("Team10/command", hostname="192.168.215.183")

		#command_received_from_microcontroller = client.on_message
		if (msg.payload == "1"):
			print("ENTERED")
			# angle_deg, distance = lt.find_vector_to_laser(dc, fov_h, res_h, res_v)
			# mf.send_motion_command(vel_msg, velocity_publisher, angle_deg, distance)
			msg.payload = 0


if __name__ == "__main__":
	main()
