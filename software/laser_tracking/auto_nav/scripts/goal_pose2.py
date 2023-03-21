#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def publishMoveBaseGoalWaitForReply(x, y):
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = "map"
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = x
  goal.target_pose.pose.position.y = y

  # to send orientation with a yaw we need quaternion transform
  #x , y, z, w = tf.transformations.quaternion_from_euler(0, 0, yaw)
  goal.target_pose.pose.orientation.x = 0
  goal.target_pose.pose.orientation.y = 0
  goal.target_pose.pose.orientation.z = 0
  goal.target_pose.pose.orientation.w = 0
  now = rospy.get_rostime()
#  print "[%i.%i] PubMove: %s x,y,z,w of %f %f %f %f yaw %f"
#    (now.secs,now.nsecs,comment,x,y,z,w,yaw)

  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  print("pre wait")
  client.wait_for_server()
  print("post wait")
  # publish the goal to the topic
  client.send_goal(goal)

  now = rospy.get_rostime()
#  print "[%i.%i] Waiting for result ..." 
  wait = client.wait_for_result()
  if not wait:
    rospy.logerr("Action server not available!")
    rospy.signal_shutdown("Action server not available!")
  else:
    now = rospy.get_rostime()
 #   print "[%i.%i] Received result" 
 #   return client.get_result()

if __name__ == '__main__':
  rospy.init_node('goal_pose_testing')
  publishMoveBaseGoalWaitForReply(10, 10)


        

