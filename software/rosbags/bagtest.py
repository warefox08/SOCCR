import rosbag
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image


depth_msg = None

topics=[]

n=1

for topic, msg, t in rosbag.Bag('test_3_5.bag').read_messages():

  if topic not in topics:
    topics.append(topic)
  # print(topic)

  if topic == "/camera1/color/image_raw":
    depth_msg = msg
    # print(msg)
  n=n+1

for topic in topics:
  print(topic)

bridge = CvBridge()
depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
depth_array = np.array(depth_image, dtype=np.uint8)
print(depth_array)
print(np.size(depth_array))
print(depth_array.ndim)
print(depth_array.shape)
print(depth_array.dtype)


# print(depth_msg)
# print(depth_msg)
# depth_test = np.uint16(np.float32(depth_msg))
# print(depth_test)
# summed_depth = depth_msg[::2]+[i*256 for i in depth_msg[1::2]]
# print(summed_depth)

# 16UC1


####
# color/camera_info -> depth/camera_info -> color/image_raw -> depth/image_raw