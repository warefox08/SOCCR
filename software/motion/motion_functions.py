#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def import_test():
	print("import successful")
	return 1, "2"

def stop(vel_msg, velocity_publisher):
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x=0
	vel_msg.angular.y=0
	vel_msg.angular.z=0
	velocity_publisher.publish(vel_msg)

def init():
	# rospy.init_node('motion_commands', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #"turtle1"???
	vel_msg = Twist()
	stop(vel_msg, velocity_publisher)
	return velocity_publisher, vel_msg #returns a tuple
	

def rotate(vel_msg, velocity_publisher, angle_deg):
	angular_speed = np.pi/2 #180deg in 2s
	angle = angle_deg*np.pi/180
	if (angle>=0): #positive angle therefore CCW rotation
		vel_msg.angular.z = abs(angular_speed)
	else: #negative angle therefore CW roation
		vel_msg.angular.z = -abs(angular_speed)
	t0 = rospy.Time.now().to_sec()
	current_angle = 0
	while (current_angle < angle):
		velocity_publisher.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)
	stop(vel_msg, velocity_publisher)
	# rospy.spin() #???

def move(vel_msg, velocity_publisher, distance):
	speed = 0.3 #m/s
	vel_msg.linear.x = abs(speed)
	#while not rospy.is_shutdown(): #why this? difference to rospy.spin() in rotate?
        #Setting the current time for distance calculus
	t0 = rospy.Time.now().to_sec()
	current_distance = 0
        #Loop to move the turtle in an specified distance
	while(current_distance < distance):
            #Publish the velocity
		velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
		t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
		current_distance= speed*(t1-t0)
        #After the loop, stops the robot
	stop(vel_msg, velocity_publisher)

def send_motion_command(vel_msg, velocity_publisher, angle_deg, distance): 
	stop(vel_msg, velocity_publisher)
	rotate(vel_msg, velocity_publisher, angle_deg) #wait till rotation is done? how? rospy.spin?
	#move(vel_msg, velocity_publisher, distance)

if __name__ == '__main__':
	rospy.init_node("test_node")
	velocity_publisher, vel_msg = init()
	send_motion_command(vel_msg, velocity_publisher, 30, 1)
