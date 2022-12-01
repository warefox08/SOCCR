#!/usr/bin/env python

### http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def rotate(angle_deg):
    rospy.init_node('robot_cleaner', anonymous=True) #"robot_cleaner"???
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #"turtle1"???
    vel_msg = Twist()

    angular_speed = np.pi/2 #180deg in 2s
    angle = angle_deg*np.pi/180

    #rostopic info /turtle1/cmd_vel

    #rosmsg show geometry_msgs/Twist

    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x=0
    vel_msg.angular.y=0

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

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    rospy.spin() #???

if __name__ == '__main__':
    try:
        rotate(90)
    except rospy.ROSInterruptException:
        pass

# roscore
# rosrun turtlesim turtlesim_node ##????

# rosrun turtlesim_cleaner rotate.py