#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

class listener: #Class listener that receives commands and publishes feedback to/from handheld device

   def __init__(self):
      self.flag = 0 #Initialise variables
      self.command = ""
      self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1) #Cancel publisher for canceling move_base goal
      self.feedback_pub = rospy.Publisher("/Feedback", String, queue_size=1) #Feedback publisher for sending feedback to handheld device

   def callback(self, data): #Callback for when a message is received
      self.raise_flag() #Raise flag when message is received
      self.command = data.data #Store message in command data member
      if self.command == "S": #If command is Stop
         self.cancel_pub.publish(GoalID()) #Publish cancelation of the current goal so that robot stops

   def init(self): 
      port_name = rospy.get_param('~tcp_port','tcp') #Rosserial port
      baud = int(rospy.get_param('~baud','57600')) #Rosserial baud rate
      rospy.Subscriber("Team10", String, self.callback) #Create a subscriber on the Team10 topic, with the callback attached

   def raise_flag(self):
      self.flag = 1
    
   def lower_flag(self):
     self.flag = 0

   def pub_feedback(self, message):
     self.feedback_pub.publish(message) #Publish feedback to device