#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def init():
    navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()
    goal = MoveBaseGoal()
    return navclient, goal
    # goal.target_pose.pose.position.z = 0.0
    # goal.target_pose.pose.orientation.x = 0.0
    # goal.target_pose.pose.orientation.y = 0.0
    # goal.target_pose.pose.orientation.z = 0.4
    # goal.target_pose.pose.orientation.w = 0.750

def send_command(navclient, goal, depth, distance_horizontal):
    #goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = depth
    goal.target_pose.pose.position.y = distance_horizontal

    navclient.send_goal(goal)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo (navclient.get_result())
    
if __name__ == '__main__':
    rospy.init_node("test_node")
    [navclient, goal] = init()
    send_command(navclient, goal, 0.1, 0.1)






