#!/usr/bin/env python

### http://wiki.ros.org/turtlesim/Tutorials/Moving%20in%20a%20Straight%20Line
import rospy
from geometry_msgs.msg import Twist

def move(distance):
    rospy.init_node('motion_commands', anonymous=True) #"robot_cleaner"???
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #"turtle1"???
    vel_msg = Twist()

    speed = 0.3 #m/s

    vel_msg.linear.x = abs(speed)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

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
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        move(2)
    except rospy.ROSInterruptException: 
        pass