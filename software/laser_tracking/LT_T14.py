#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
#from PIL import Image
from sensor_msgs.msg import Image

def init():
    # Initialize Camera 
    fov_h = 87
    res_v = 480 #720
    res_h = 640 #1280
    # deg_per_pixel_h = fov_h/res_h
    # origin = [res_h/2, res_v/2]
    return fov_h, res_h, res_v

def color_callback(img_msg):
    rospy.loginfo(img_msg.header)
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        return color_image
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

def depth_callback(img_msg):
    rospy.loginfo(img_msg.header)
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        return depth_image
    except CvBridgeError:
        rospy.logerr("CvBridge Error")


def search_for_laser(fov_h, res_h):

# to - do 
# fix how the subscriber is set up 
# add a publisher to publish coords
    depth_msg = None
    color_msg = None

    depth_sub = rospy.Subscriber("/camera1/aligned_depth_to_color/image_raw", Image, depth_callback)
    color_sub = rospy.Subscriber("/camera1/color/image_raw", Image, color_callback)

    color_image = # how do i get this from callback? 

    color_image_BGR = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    depth_image = 

    color_frame = np.float32(color_image)

    # Convert the BGR (opencv does BGR instead of RGB) color frame to a HSV frame 
    hsv_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Define range of red color in HSV -> red hue boundary -- worth testing out and messing around with 
    lower_red = np.array([30, 150, 50])
    upper_red = np.array([255, 255, 180])

    # print(color_frame)

    cv2.imshow('color_image', color_image)
    cv2.imshow('hsv_frame', hsv_frame)

    # Create a mask using the HSV range
    mask_colour = cv2.inRange(hsv_frame, lower_red, upper_red)
    # cv2.imshow('mask_colour', mask_colour)

    # Threshold the HSV image to filter out all 'non-red' pixels
    red_filter = cv2.bitwise_and(color_frame, color_frame, mask = mask_colour)
    # cv2.imshow('red_filter', red_filter) ###Moved to show_data()

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask_colour) #finds the min, max and index values of the mask - useful data for mask tuning

    # Make a binarised version of the image and find the coordinates of non-zero pixels 

    img = cv2.cvtColor(red_filter,cv2.COLOR_BGR2GRAY)
    img = img.astype(np.uint8) #converts the image to int type - each pixel has an int value of 0-255 rathe than floating points between 0 and 
    points = cv2.findNonZero(img) #finds coordinates of non-zero pixels
    avg = np.mean(points, axis=0) #find the average x and y of all red pixels 
    # take the integer values of the average coordinate to later find the distance of the specified pixel
    p1 = int(avg[0][0])
    p2 = int(avg[0][1])

    # The color frame is 640x480, with 0,0 at the top left pixel, reset 0,0 to the centre of the frame by translation
    # defining origin and angular resolution if needed:
    #p1_o = p1-320
    #p2_o = -p2+240
    #deg_per_pixel_h = fov_h/res_h

    depth_pixel = depth_frame[p2, p1]
    dist_pixel = depth_frame.get_distance(p1, p2)

# next section leverages the cameras own functions - but requires pyrealsense

    #point_3d = np.array([0, 0, 0], ndmin=3)
    #depth_origin = depth_frame[240, 320]
    #depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

    #point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dist_pixel) # should be used to get 3d world coords BUT requires pyrealsense 
    #x_m = point_3d[0][0]
    #y_m = point_3d[0][1]
    #z_m = point_3d[0][2]

# Using simple trig - 
    x_abs = np.sqrt((dist_pixel*dist_pixel)*(depth_pixel*depth_pixel))
    if p1 >= 320:
        x_m = x_abs
    else:
        x_m = -1*x_abs 
    y_m = depth_pixel 
    z_m = 0
    return x_m, y_m, z_m


# def find_laser_pixel(points, fov_h, res_h, depth_frame):

def show_data(red_filter, color_frame, p1, p2, distance_x, distance_origin, angle_deg_h):
    cv2.imshow('red_filter', red_filter)
    cv2.circle(color_frame, (p1,p2), 10, (255, 0, 0)) # creates a blue circle of diameter '10' around the average laser coordinate
    cv2.imshow("Color frame", color_frame)

def log_data(x, y, z):
    write_string = " x: " + str(x) + " y: "+ str(y) + " z: "+ str(z) + "\n" #String to be written to txt file
    with open('data.txt', 'w') as f:
        f.write(write_string)

def find_laser_coordinates():
    laser_found = 0
    while(not laser_found):
        [laser_found, x, y, z] = search_for_laser(fov_h, res_h)
        key = cv2.waitKey(1)
        if key == 27: # esc key to break 
            break
    return x, y, z


if __name__ == "__main__":
    [fov_h, res_h, res_v] = init()
    try:
        while True:
            x_d, y_d, z_d = find_laser_coordinates(fov_h, res_h, res_v)
            print("x: " + str(x_d)) + " y: " + str(y_d) + " z: " + str(z_d)
    except KeyboardInterrupt:
        log_data(x_d, y_d, z_d)
        print("data logged") 