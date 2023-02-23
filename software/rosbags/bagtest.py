import rosbag
import rospy
import numpy as np

# c_info_topic = None

n=1
# with rosbag.Bag("test_3.bag") as bag:
for topic, msg, t in rosbag.Bag('test_3.bag').read_messages():
  if n==1:
    c_info_topic = topic
    c_info_msg = msg
    c_info_t = t
  if n==2:
    d_info_topic = topic
    d_info_msg = msg
    d_info_t = t
  if n==3:
    c_raw_topic = topic
    c_raw_msg = msg
    c_raw_t = t
  if n==4:
    d_raw_topic = topic
    d_raw_msg = msg
    d_raw_t = t
  n=n+1

# print(c_info_topic)
# print(c_info_msg)
# print(d_info_topic)
# print(d_info_msg)

print(len(d_raw_msg.data))
print(d_raw_msg.height)
print(d_raw_msg.width)




####
# color/camera_info -> depth/camera_info -> color/image_raw -> depth/image_raw