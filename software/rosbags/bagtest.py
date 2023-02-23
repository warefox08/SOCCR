import rosbag
import rospy

with rosbag.Bag("test_3.bag") as bag:
        for topic, msg, t in rosbag.Bag('input.bag').read_messages():
          print(topic)
          print(msg)
          print(t)