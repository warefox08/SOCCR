#!/usr/bin/env python3
import rospy
import goal_pose_copy_2 as gp

rospy.init_node("goal_pose_testing")
gp.move_command(10,10)
