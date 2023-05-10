#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callbacks definition

def move_command(x,y, rot_z, rot_w):

  navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction) #Initialise navigation client
  navclient.wait_for_server() #Wait until command is sent

  goal = MoveBaseGoal() #Create a goal object
  goal.target_pose.header.frame_id = "base_link" #Set goal relative to base_link frame
  goal.target_pose.header.stamp = rospy.Time.now()

  goal.target_pose.pose.position.x = x
  goal.target_pose.pose.position.y = y
  goal.target_pose.pose.position.z = 0.0
  goal.target_pose.pose.orientation.x = 0
  goal.target_pose.pose.orientation.y = 0
  goal.target_pose.pose.orientation.z = rot_z
  goal.target_pose.pose.orientation.w = rot_w
  
  print("before send_goal")
  navclient.send_goal(goal)
  print("after send_goal")
  finished = navclient.wait_for_result()
  print("finished")
  if not finished:
    rospy.logerr("Action server not available!")
  else:
    rospy.loginfo ( navclient.get_result())