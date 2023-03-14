#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class laser_tracker:

    def __init__(self):
        # Initialize Camera 
        self.fov_h = 87
        self.res_v = 480 #720
        self.res_h = 640 #1280
        self.bridge = CvBridge()
        self.depth_msg = None
        self.color_msg = None
        self.depth_sub = rospy.Subscriber("/camera1/aligned_depth_to_color/image_raw", Image, self.image_callback, (self.depth_msg))
        self.color_sub = rospy.Subscriber("/camera1/color/image_raw", Image, self.image_callback, (self.color_msg))
        

    def image_callback(self, img_msg, args):
        rospy.loginfo(img_msg.header)
        try:
            args[0] = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
            

    def search_for_laser(self):

        color_image = cv2.cvtColor(self.color_msg, cv2.COLOR_RGB2BGR)

        color_frame = np.float32(color_image)

        # Convert the BGR (opencv does BGR instead of RGB) color frame to a HSV frame 
        hsv_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define range of red color in HSV -> red hue boundary -- worth testing out and messing around with 
        lower_red = np.array([30, 150, 50])
        upper_red = np.array([255, 255, 180])

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
        cv2.imshow ('img', img)
        points = cv2.findNonZero(img) #finds coordinates of non-zero pixels 

        if (points is not None):
            [angle, distance, distance_x] = self.find_laser_cords(points, color_frame, red_filter)
            #print("laser found")
            return 1, angle, distance, distance_x
        else:
            # print("laser not found")
            return 0, None, None, None

    def find_laser_cords(self, points):
        avg = np.mean(points, axis=0) #find the average x and y of all red pixels 
        # take the integer values of the average coordinate to later find the distance of the specified pixel
        p1 = int(avg[0][0])
        p2 = int(avg[0][1])

        # The color frame is 640x480, with 0,0 at the top left pixel, reset 0,0 to the centre of the frame by translation
        p1_o = p1-320
        p2_o = -p2+240
        deg_per_pixel_h = self.fov_h/res_h
        angle_deg_h = (p1-320)*deg_per_pixel_h

        distance = self.depth_msg[p2, p1] #finds the distance to the laser point *NOTE: Distance = depth from camera Plane 
        distance_origin = self.depth_msg[240, 320] #depth of origin pixel 

        angle_rad = np.pi*angle_deg_h/180
        distance_x = distance_origin/np.cos(angle_rad)
        
        angle = np.arccos(distance_origin / distance_x) # is this not just returning angle_rad again?
        angle_d = 180*angle/np.pi

        # Math for the generalised version -- NOTE: THIS IS UNTESTED 
        #### care for diag shouldnt this be (l/sin b) or (l/cos theta) since its all in the same plane elevation 
        #### shouldnt be taken into consideration
        l = distance_origin
        a = angle_rad 
        b = (np.pi/2)-a # angle beta = 90 - alpha (fov_h angle) i.e pi/2 - alpha (in rad) 
        diag = (l)*np.sin(b) # The 'diagonal' distance to the laser point as a function using the sine rule
        w = (l)*np.sin(a) # The 'width'i.e. distance between the origin point and the laser point (alt to distance_x)
        
        # log_data(p1, p2, distance_x, angle_deg_h)

        return angle_deg_h, distance, distance_x

    def find_vector_to_laser(self):
        laser_found = 0
        while(not laser_found):
            [laser_found, angle, distance, distance_x] = self.search_for_laser()
            key = cv2.waitKey(1)
            if key == 27: # esc key to break 
                break
        return angle, distance, distance_x


if __name__ == "__main__":
    tracker = laser_tracker()
    while True:
        angle, distance, distance_x = tracker.find_vector_to_laser()
        print("a: " + str(angle) + " d: " + str(distance))
