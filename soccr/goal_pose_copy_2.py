#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callbacks definition

def move_command(x,y, rot_z, rot_w, frame):

  navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
  navclient.wait_for_server()

# Example of navigation goal
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = frame 
  goal.target_pose.header.stamp = rospy.Time.now()

  goal.target_pose.pose.position.x = x
  goal.target_pose.pose.position.y = y
  goal.target_pose.pose.position.z = 0.0
  goal.target_pose.pose.orientation.x = 0
  goal.target_pose.pose.orientation.y = 0
  goal.target_pose.pose.orientation.z = rot_z
  goal.target_pose.pose.orientation.w = rot_w
  
  navclient.send_goal(goal)
  finished = navclient.wait_for_result()
  print("finished")
  if not finished:
    rospy.logerr("Action server not available!")
  else:
    rospy.loginfo ( navclient.get_result())

if __name__=="__main__":
  rospy.init_node("test_node")
  move_command(1,-0.5)
