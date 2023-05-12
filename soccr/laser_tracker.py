#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class laser_tracker:

    def __init__(self):
        self.fov_h = 69
        self.res_v = 480 #720
        self.res_h = 640 #1280
        self.f_x = 602.04
        self.f_y = 601.984
        self.c_x = 327.888
        self.c_y = 239.312
        self.depth_flag = 0
        self.color_flag = 0
        self.depth_sub = rospy.Subscriber("/camera1/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.color_sub = rospy.Subscriber("/camera1/color/image_raw", Image, self.color_callback)
        self.frame_counter = 0
    
    def color_callback(self, img_msg):
        #rospy.loginfo(img_msg.header)
        bridge = CvBridge()
        if not self.color_flag:
            self.color_flag = 1
            print("color flag raised")
        try:
            #print ("gets into color callback")
            self.frame_counter = self.frame_counter + 1
            self.color_msg = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

        except CvBridgeError:
            rospy.logerr("CvBridge Error")

    def depth_callback(self, img_msg):
        #rospy.loginfo(img_msg.header)
        bridge = CvBridge()
        if not self.depth_flag:
            self.depth_flag = 1
            print("depth flag raised")
        try:
            #print ("gets into depth callback")
            self.depth_msg = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            
            #  #print (self.depth_msg)
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
            
    def search_for_laser(self):
        color_image = cv2.cvtColor(self.color_msg, cv2.COLOR_RGB2BGR)

        # Convert the BGR (opencv does BGR instead of RGB) color frame to a HSV frame 
        hsv_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    
        # Thresholding for blue laser
        l_blue = np.array([105, 70, 225])
        u_blue = np.array([120, 255, 255])
        # Blue mask
        blue_mask = cv2.inRange(hsv_frame, l_blue, u_blue)
        # Blue filter
        blue_filter = cv2.bitwise_and(color_image, color_image, mask = blue_mask)
        #minMaxLoc?

        # Make a binarised version of the image and find the coordinates of non-zero pixels 
        img = cv2.cvtColor(blue_filter,cv2.COLOR_BGR2GRAY)
        img = img.astype(np.uint8) #converts the image to int type - each pixel has an int value of 0-255 rathe than floating points between 0 and 
        points = cv2.findNonZero(img) #finds coordinates of non-zero pixels 

        # Debug imshows - for intermediate frames 
        #cv2.imshow("hsv", hsv_frame)
        #cv2.imshow('blue', blue_filter)
        #cv2.imshow('and', img)
    
        # if laser is picked up, find its coordinates 
        if (points is not None):
            [angle, distance_x, distance_y, distance_z] = self.find_laser_cords(points,color_image)
            return 1, angle, distance_x, distance_y, distance_z
        else:
            print ("points is None")
            #cv2.imshow("color_image", color_image)
            #cv2.waitKey(0) # frame by frame for debugging A
            return 0, None, None, None, None

    def find_laser_cords(self, points, color_image):
        avg = np.mean(points, axis=0) #find the average x and y of all red pixels 
        # take the integer values of the average coordinate to later find the distance of the specified pixel
        p1 = int(avg[0][0])
        p2 = int(avg[0][1])

        # The color frame is 640x480, with 0,0 at the top left pixel, reset 0,0 to the centre of the frame by translation
        p1_o = p1-320
        p2_o = -p2+240
        deg_per_pixel_h = self.fov_h/self.res_h
        angle_deg_h = (p1-320)*deg_per_pixel_h

        distance_x = self.depth_msg[p2, p1] #finds the distance to the laser point *NOTE: Distance = depth from camera Plane 
        distance_origin = self.depth_msg[240, 320] #depth of origin pixel 

        # distance_x = -(np.sin(angle_rad)*(distance/np.sin(phi_rad)))
        distance_y = -(distance_x*((p1-self.c_x)/self.f_x))
        distance_z = -(distance_x*((p2-self.c_y)/self.f_y))

        # log_data(p1, p2, distance_x, angle_deg_h)
        self.show_data(color_image, p1, p2, distance_x, distance_y, distance_z) # was returning distance_origin ?

        return angle_deg_h, distance_x, distance_y, distance_z
    
    def show_data(self, color_image, p1, p2, d_x, d_y, d_z):
        # show frames for debugging 
        #cv2.imshow("red", red_filter)
        #cv2.imshow("hsv", hsv_frame)
        #cv2.imshow("binary image", img)
        #cv2.imshow('red_filter', red_filter)
        cv2.circle(color_image, (p1,p2), 10, (0, 0, 255)) # creates a blue circle of diameter '10' around the average laser coordinate
        cv2.putText(color_image, "x: {:.1f} mm".format(d_x), (50, 350), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.putText(color_image, "y: {:.1f} mm".format(d_y), (50, 400), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.putText(color_image, "z: {:.1f} mm".format(d_z), (50, 450), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        #cv2.putText(color_image, "pixel_x: {}".format(p1), (50, 300), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.imshow("color_image", color_image)
        #cv2.circle(color_image, (p1,p2), 10, (255, 0, 0))
        print("\nPixel_x: ", p1)
        #cv2.imshow("depth", self.depth_msg)
        cv2.waitKey(0) # frame by frame for debugging 

    def find_vector_to_laser(self):
        laser_found = 0
        while(not laser_found):
            [laser_found, angle, distance_x, distance_y, distance_z] = self.search_for_laser()
            key = cv2.waitKey(1)
            if key == 27: # esc key to break 
                break
        return angle, distance_x, distance_y, distance_z


if __name__ == "__main__":
    rospy.init_node('test')
    tracker = laser_tracker()
    try:
        #pass
        while True:
            if (tracker.color_flag and tracker.depth_flag):
                angle, distance_x, distance_y, distance_z = tracker.find_vector_to_laser()
                print("\nd_x: ", distance_x)
                print("\nd_y: ", distance_y)
                print("\nd_z: ", distance_z)
                #rospy.sleep(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        # pass
        print("hell escaped") 