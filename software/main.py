from lzma import MF_BT3
import sys, os
sys.path.append(os.path.join(os.getcwd(), "motion")) #add function folder to path
sys.path.append(os.path.join(os.getcwd(), "laser_tracking")) #add function folder to path
import motion_functions as mf
import laser_tracking_functions as lt
# import laser_tracking_wip as lt

def main():
	debug=0
	if debug==1:
		[test1, test2] = mf.import_test
		print(test1)
		print(test2)
	
	velocity_publisher, vel_msg = mf.init()

	command_received_from_microcontroller = 0
	while(1):
		if (command_received_from_microcontroller):
			angle_deg, distance = lt.find_vector_to_laser()	### RECEIVE COMMAND FROM LASER TRACKER
			mf.send_motion_command(vel_msg, velocity_publisher, angle_deg, distance)
			command_received_from_microcontroller = 0


if __name__ == "__main__":
	main()