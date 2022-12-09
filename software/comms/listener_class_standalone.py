#!/usr/bin/env python
import rospy
from rosserial_python import SerialClient, RosSerialServer
from std_msgs.msg import String

class listener:

   def __init__(self):
      self.flag = 0

   def callback(self, arg):
      self.raise_flag()
      print("Message Received")

   def init(self):
      #rospy.init_node("Team10")
      port_name = rospy.get_param('~tcp_port','tcp')
      baud = int(rospy.get_param('~baud','57600'))
      rospy.Subscriber("Team10", String, self.callback)

   def raise_flag(self):
     self.flag = 1
    
   def lower_flag(self):
     self.flag = 0

    

   
    

# if __name__ == '__main__':
#     init()