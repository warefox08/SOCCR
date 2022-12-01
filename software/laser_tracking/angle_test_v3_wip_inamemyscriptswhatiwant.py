import cv2
import pyrealsense2
from realsense_depth import * #dependency --> realsense_depth.py, no other files/ scripts necessary 


# Initialize Camera Intel Realsense
dc = DepthCamera()

while True: #continuos loop unless break condition met 
    ret, depth_frame, color_frame = dc.get_frame() #returns the depth frame and color frame fed from the camera 
    
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
    cv2.imshow('red_filter', red_filter)

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask_colour) #finds the min, max and index values of the mask - useful data for mask tuning

    # Make a binarised version of the image and find the coordinates of non-zero pixels 

    img = cv2.cvtColor(red_filter,cv2.COLOR_BGR2GRAY)
    img = img.astype(np.uint8) #converts the image to int type - each pixel has an int value of 0-255 rathe than floating points between 0 and 
    points = cv2.findNonZero(img) #finds coordinates of non-zero pixels 

    #avg = np.array([])
    
    # Find the Average Coordinate of the red pixels
    
    if (points is not None): #points is NoneType if there is no laser 
        avg = np.mean(points, axis=0) #find the average x and y of all red pixels 
        print ('The average coordinate is:')
        print (avg)

        # take the integer values of the average coordinate to later find the distance of the specified pixel
        p1 = int(avg[0][0])
        p2 = int(avg[0][1])
        # print out the x,y coordinate
        print ('x coordinate is:')
        print (p1)
        print ('y coordinate is:')
        print (p2)

        # The color frame is 640x480, with 0,0 at the top left pixel, reset 0,0 to the centre of the frame by translation
        p1_o = p1-320
        p2_o = -p2+240

    # Quik Mafs
    
    #NOTE: The current working version uses the math in this section which is applicable to the basic use case 

        # The d435i has an FOV of 87 degrees, which is used to find the degree of rotation about the vertical axis represented by the x translation
        deg_per_pixel = 87/640
        angle_fov = (p1-320)*deg_per_pixel

        
        distance = depth_frame[p2, p1] #finds the distance to the laser point *NOTE: Distance = depth from camera Plane 
        distance_origin = depth_frame[240, 320] #depth of origin pixel 

        angle_rad = np.pi*angle_fov/180
        distance_x = distance_origin/np.cos(angle_rad)
        

        angle = np.arccos(distance_origin / distance_x) # is this not just returning angle_rad again?
        angle_d = 180*angle/np.pi

    # Math for the generalised version -- NOTE: THIS IS UNTESTED 

        l = distance_origin
        a = angle_rad 
        b = (np.pi/2)-a # angle beta = 90 - alpha (fov angle) i.e pi/2 - alpha (in rad) 
        diag = (l)*np.sin(b) # The 'diagonal' distance to the laser point as a function using the sine rule
        w = (l)*np.sin(a) # The 'width'i.e. distance between the origin point and the laser point (alt to distance_x)

    # -- ALSO NEED TO DOUBLE CHECK THE MATH and all working assumptions/ use cases

        # Relevant output to live feed (all available fonts are hershey :\ )

        cv2.circle(color_frame, (p1,p2), 10, (255, 0, 0)) # creates a blue circle of diameter '10' around the average laser coordinate

        cv2.putText(color_frame, "x: {}".format(p1-320), (50, 400), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        
        cv2.putText(color_frame, "y: {}".format(-p2+240), (50, 450), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.putText(color_frame, "d_x: {}".format(distance_x), (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        
        cv2.putText(color_frame, "d_o: {}mm".format(distance_origin), (320, 240), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.putText(color_frame, "ang: {}".format(angle_fov), (320, 350), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.putText(color_frame, "y: {}".format(-p2+240), (100, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.putText(color_frame, "dx: {}mm".format(distance_x), (200, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        #Display the color frame

        cv2.imshow("Color frame", color_frame)

        # Writing data (last line) out to a text file

        write_string = "x: " + str(p1) + " y: " + str(p2) + " d: " + str(distance) + "\n" #String to be written to txt file

        write_string = "x: " + str(p1-320) + " y: " + str(-p2+240) + " d: " + str(distance) + "\n" #String to be written to txt file (0,0 at centre of screen)

        with open('data.txt', 'w') as f:
            f.write(write_string)


    # loop or break
    key = cv2.waitKey(1) #milisecond delay before going to next frame 
    if key == 27: # esc key to break 
        break
