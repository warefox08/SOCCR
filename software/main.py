from lzma import MF_BT3
import sys, os
sys.path.append(os.getcwd() + "\\motion") #add function folder to path
sys.path.append(os.getcwd() + "\\laser_tracking") #add function folder to path
import motion_functions as mf
import laser_tracking as lt

def main():
	debug=1
	if debug==1:
		[test1, test2] = mf.import_test
		print(test1)
		print(test2)
	
	velocity_publisher, vel_msg = mf.init()
	angle_deg, distance = lt.find_laser()	### RECEIVE COMMAND FROM LASER TRACKER
	mf.send_motion_command(vel_msg, velocity_publisher, angle_deg, distance)


if __name__ == "__main__":
	main()