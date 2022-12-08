#!/usr/bin/env python
import rospy
from rosserial_python import SerialClient, RosSerialServer
from std_msgs.msg import String
def callback(arg):
    print("Message Received")

def init():
    rospy.init_node("Team10")
    port_name = rospy.get_param('~tcp_port','tcp')
    baud = int(rospy.get_param('~baud','57600'))
    rospy.Subscriber("Team10", String, callback)
    

if __name__ == '__main__':
    init()