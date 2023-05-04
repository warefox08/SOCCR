#!/usr/bin/env python
import rospy
from rosserial_python import SerialClient, RosSerialServer
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

class listener:

   def __init__(self):
      self.flag = 0
      self.angle = 0
      self.distance = 0
      self.command = ""
      self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
#      self.feedback_pub = rospy.Publisher("/Feedback", String, queue_size=1)
      # port_name = rospy.get_param('~tcp_port','tcp')
      # baud = int(rospy.get_param('~baud','57600'))
      # rospy.Subscriber("Team10", String, self.callback)

   def callback(self, data):
      self.raise_flag()
      self.command = data.data
      print(self.command)
      print("Message Received")
      if self.command == "S":
         self.cancel_pub.publish(GoalID())
         print("stop received")
      #self.read_data()

   def init(self):
      # rospy.init_node("Team10")
      port_name = rospy.get_param('~tcp_port','tcp')
      baud = int(rospy.get_param('~baud','57600'))
      rospy.Subscriber("Team10", String, self.callback)
      # rostopic pub Team10 software/comms

   def raise_flag(self):
      self.flag = 1
    
    
   def lower_flag(self):
     self.flag = 0

   def pub_feedback(self, message):
     self.feedback_pub.publish(message)

   def read_data(self):
      with open("data.txt") as f:
         data = f.read()
      split_data    = data.split(":")
      self.distance = split_data[0] 
      self.angle    = split_data[1]
    

# if __name__ == '__main__':
#     init()
