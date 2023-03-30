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
        self.depth_flag = 0
        self.color_flag = 0
        self.depth_sub = rospy.Subscriber("/camera1/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.color_sub = rospy.Subscriber("/camera1/color/image_raw", Image, self.color_callback)
        #rospy.spin()
        #rospy.sleep(5)
    
    def color_callback(self, img_msg):
        #rospy.loginfo(img_msg.header)
        bridge = CvBridge()
        if not self.color_flag:
            self.color_flag = 1
        try:
            #print ("gets into color callback")
            self.color_msg = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

        except CvBridgeError:
            rospy.logerr("CvBridge Error")

    def depth_callback(self, img_msg):
        #rospy.loginfo(img_msg.header)
        bridge = CvBridge()
        if not self.depth_flag:
            self.depth_flag = 1
        try:
            #print ("gets into depth callback")
            self.depth_msg = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            
            #  #print (self.depth_msg)
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
            
    def search_for_laser(self):
        color_image = cv2.cvtColor(self.color_msg, cv2.COLOR_RGB2BGR)

        color_frame = np.float32(color_image)

        # Convert the BGR (opencv does BGR instead of RGB) color frame to a HSV frame 
        hsv_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define range of red color in HSV -> red hue boundary -- worth testing out and messing around with 
        lower_red = np.array([160, 70, 90]) #([30, 150, 50]) - these vals included blue and red 
        upper_red = np.array([180, 255, 255]) #([255, 255, 180])

        # Create a mask using the HSV range
        mask_colour = cv2.inRange(hsv_frame, lower_red, upper_red)
        # Threshold the HSV image to filter out all 'non-red' pixels
        #red_filter = cv2.bitwise_and(color_frame, color_frame, mask = mask_colour)
        red_filter = cv2.bitwise_and(color_image, color_image, mask = mask_colour)

        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask_colour) #finds the min, max and index values of the mask - useful data for mask tuning
        
        # Make a binarised version of the image and find the coordinates of non-zero pixels 

        img = cv2.cvtColor(red_filter,cv2.COLOR_BGR2GRAY)
        img = img.astype(np.uint8) #converts the image to int type - each pixel has an int value of 0-255 rathe than floating points between 0 and 

        points = cv2.findNonZero(img) #finds coordinates of non-zero pixels 
    

        if (points is not None):
            [angle, distance, distance_x] = self.find_laser_cords(points,color_image)
            return 1, angle, distance, distance_x
        else:
            return 0, None, None, None

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
        phi = 90-angle_deg_h

        distance = self.depth_msg[p2, p1] #finds the distance to the laser point *NOTE: Distance = depth from camera Plane 
        distance_origin = self.depth_msg[240, 320] #depth of origin pixel 

        angle_rad = np.pi*angle_deg_h/180
        phi_rad = np.pi*phi/180
        # distance_x = np.tan(angle_rad)*distance_origin
        distance_x = -(np.sin(angle_rad)*(distance/np.sin(phi_rad)))
        
        # angle = np.arccos(distance_origin / distance_x) # is this not just returning angle_rad again?
        # angle_d = 180*angle/np.pi

        # Math for the generalised version -- NOTE: THIS IS UNTESTED 
        #### care for diag shouldnt this be (l/sin b) or (l/cos theta) since its all in the same plane elevation 
        #### shouldnt be taken into consideration
        # l = distance_origin
        # a = angle_rad 
        # b = (np.pi/2)-a # angle beta = 90 - alpha (fov_h angle) i.e pi/2 - alpha (in rad) 
        # diag = (l)*np.sin(b) # The 'diagonal' distance to the laser point as a function using the sine rule
        # w = (l)*np.sin(a) # The 'width'i.e. distance between the origin point and the laser point (alt to distance_x)
        
        # log_data(p1, p2, distance_x, angle_deg_h)
        #self.show_data(color_image, p1, p2, distance_origin, distance_x)

        return angle_deg_h, distance_origin, distance_x
    
    def show_data(self, color_image, p1, p2, d_x, d_y):
        # show frames for debugging 
        #cv2.imshow("red", red_filter)
        #cv2.imshow("hsv", hsv_frame)
        #cv2.imshow("binary image", img)
        #cv2.imshow('red_filter', red_filter)
        cv2.circle(color_image, (p1,p2), 10, (255, 0, 0)) # creates a blue circle of diameter '10' around the average laser coordinate
        cv2.putText(color_image, "x: {}".format(d_x), (50, 400), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.putText(color_image, "y: {}".format(d_y), (50, 450), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.imshow("color_image", color_image)
        #cv2.circle(color_image, (p1,p2), 10, (255, 0, 0))
        #cv2.waitKey(0) # frame by frame for debugging 

    def find_vector_to_laser(self):
        laser_found = 0
        while(not laser_found):
            [laser_found, angle, distance, distance_x] = self.search_for_laser()
            key = cv2.waitKey(1)
            if key == 27: # esc key to break 
                break
        return angle, distance, distance_x


if __name__ == "__main__":
    rospy.init_node('test')
    tracker = laser_tracker()
    try:
        #pass
        while True:
            if (tracker.color_flag and tracker.depth_flag):
                angle, distance_x, distance_y = tracker.find_vector_to_laser()
                print("\nd_x: ", distance_x)
                print("d_y: ", distance_y)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        # pass
        print("hell escaped") 