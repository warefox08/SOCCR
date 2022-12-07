#!/usr/bin/env python
import rospy
from rosserial_python import SerialClient, RosSerialServer
from std_msgs.msg import String

def init():
    rospy.init_node("listen")
    rospy.loginfo("ROS Serial Listener")
    port_name = rospy.get_param('~tcp_port','TCP')
    baud = int(rospy.get_param('~baud','57600'))


if __name__ == '__main__':
    init()