#!/usr/bin/env python
import rospy
from rosserial_python import SerialClient, RosSerialServer
from std_msgs.msg import String
def callback(data,flag):
    print("Message Received")
    print(flag)
    flag = 1
    print(flag)

def init(flag):
    rospy.init_node("Team10")
    port_name = rospy.get_param('~tcp_port','tcp')
    baud = int(rospy.get_param('~baud','57600'))
    rospy.Subscriber("Team10", String, callback, flag)
    

if __name__ == '__main__':
    init()