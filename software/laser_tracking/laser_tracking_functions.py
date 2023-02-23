#!/usr/bin/env python
import cv2
import numpy as np

def init():
    # Initialize Camera 
    fov_h = 87
    res_v = 480 #720
    res_h = 640 #1280
    # deg_per_pixel_h = fov_h/res_h
    # origin = [res_h/2, res_v/2]
    return fov_h, res_h, res_v

def search_for_laser(fov_h, res_h):
    
    with open("rosbag.txt") as f:
        data = f.read()
    
    depth_frame = data_depth...
    color_frame = data_color...

    # Convert the BGR (opencv does BGR instead of RGB) color frame to a HSV frame 
    hsv_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

    # Define range of red color in HSV -> red hue boundary -- worth testing out and messing around with 
    lower_red = np.array([30, 150, 50])
    upper_red = np.array([255, 255, 180])

    # Create a mask using the HSV range
    mask_colour = cv2.inRange(hsv_frame, lower_red, upper_red)
    #cv2.imshow('mask_colour', mask_colour)

    # Threshold the HSV image to filter out all 'non-red' pixels
    red_filter = cv2.bitwise_and(color_frame, color_frame, mask = mask_colour)
    # cv2.imshow('red_filter', red_filter) ###Moved to show_data()

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask_colour) #finds the min, max and index values of the mask - useful data for mask tuning

    # Make a binarised version of the image and find the coordinates of non-zero pixels 

    img = cv2.cvtColor(red_filter,cv2.COLOR_BGR2GRAY)
    img = img.astype(np.uint8) #converts the image to int type - each pixel has an int value of 0-255 rathe than floating points between 0 and 
    points = cv2.findNonZero(img) #finds coordinates of non-zero pixels 

    if (points is not None):
        [angle, distance] = find_laser_cords(points, fov_h, res_h, depth_frame, color_frame, red_filter)
        return 1, angle, distance
    else:
        return 0, None, None
    #### Moved to find_vector_to_laser()
    # key = cv2.waitKey(1)
    # if key == 27: # esc key to break 
    #     break

def find_laser_cords(points, fov_h, res_h, depth_frame, color_frame, red_filter):
    avg = np.mean(points, axis=0) #find the average x and y of all red pixels 
    # take the integer values of the average coordinate to later find the distance of the specified pixel
    p1 = int(avg[0][0])
    p2 = int(avg[0][1])

    # The color frame is 640x480, with 0,0 at the top left pixel, reset 0,0 to the centre of the frame by translation
    p1_o = p1-320
    p2_o = -p2+240
    deg_per_pixel_h = fov_h/res_h
    angle_deg_h = (p1-320)*deg_per_pixel_h

    distance = depth_frame[p2, p1] #finds the distance to the laser point *NOTE: Distance = depth from camera Plane 
    distance_origin = depth_frame[240, 320] #depth of origin pixel 

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
    
    show_data(red_filter, color_frame, p1, p2, distance_x, distance_origin, angle_deg_h)
    # log_data(p1, p2, distance_x, angle_deg_h)

    return angle_deg_h, distance_x

def show_data(red_filter, color_frame, p1, p2, distance_x, distance_origin, angle_deg_h):
    cv2.imshow('red_filter', red_filter)
    cv2.circle(color_frame, (p1,p2), 10, (255, 0, 0)) # creates a blue circle of diameter '10' around the average laser coordinate

    cv2.putText(color_frame, "x: {}".format(p1-320), (50, 400), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    
    cv2.putText(color_frame, "y: {}".format(-p2+240), (50, 450), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

    cv2.putText(color_frame, "d_x: {}".format(distance_x), (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    
    cv2.putText(color_frame, "d_o: {}mm".format(distance_origin), (320, 240), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

    cv2.putText(color_frame, "ang: {}".format(angle_deg_h), (320, 350), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

    cv2.putText(color_frame, "y: {}".format(-p2+240), (100, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

    cv2.putText(color_frame, "dx: {}mm".format(distance_x), (200, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

    #Display the color frame

    cv2.imshow("Color frame", color_frame)

def log_data(angle_deg_h, distance_x):
    write_string = " d: " + str(distance_x) + " a: "+ str(angle_deg_h) + "\n" #String to be written to txt file
    with open('data.txt', 'w') as f:
        f.write(write_string)

def find_vector_to_laser(fov_h, res_h, res_v):
    laser_found = 0
    while(not laser_found):
        [laser_found, angle, distance] = search_for_laser(fov_h, res_h)
        key = cv2.waitKey(1)
        if key == 27: # esc key to break 
            break
    return angle, distance


if __name__ == "__main__":
    [fov_h, res_h, res_v] = init()
    try:
        while True:
            angle, distance = find_vector_to_laser(dc, fov_h, res_h, res_v)
            print("a: " + str(angle) + " d: " + str(distance))
    except KeyboardInterrupt:
        log_data(angle, distance)
        print("data logged") 
