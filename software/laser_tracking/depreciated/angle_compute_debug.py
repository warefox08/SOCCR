import cv2
import pyrealsense2
from realsense_depth import *



# Initialize Camera Intel Realsense
dc = DepthCamera()

while True: #this should let us press keys to advance frames
    ret, depth_frame, color_frame = dc.get_frame()

# here we would like to threshold out the image to identify the laser and set the x y values for 'point

       # Detect laser colour
    
    # Convert BGR to HSV
    hsv_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

    #define range of red color in HSV

    #test for lower and upper bound of red hue
    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    #perfect red hue boundary
    lower_red_1 = np.array([30, 150, 50])
    upper_red_1 = np.array([255, 255, 180])

    # lower boundary RED color range values; Hue (0 - 10)
    lower_red_low = np.array([0, 100, 20])
    upper_red_low = np.array([10, 255, 255])
 
    # upper boundary RED color range values; Hue (160 - 180)
    lower_red_up = np.array([160,100,20])
    upper_red_up = np.array([179,255,255])

    lower_mask = cv2.inRange(hsv_frame, lower_red_low , upper_red_low)
    upper_mask = cv2.inRange(hsv_frame, lower_red_up, upper_red_up)
    
    full_mask = lower_mask + upper_mask
    # full_mask = upper_mask
    
    result = cv2.bitwise_and(color_frame, color_frame, mask=full_mask)
    
    cv2.imshow('result',result)


    
    #Threshold the HSV image to get only red colors

    mask_colour = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
    #cv2.imshow('mask_colour', mask_colour)

    red_filter = cv2.bitwise_and(color_frame, color_frame, mask = mask_colour)
    cv2.imshow('red_filter', red_filter)

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask_colour)

    # make a binarised version of the image and use find nonzeroes to get coordinates

    img = cv2.cvtColor(red_filter,cv2.COLOR_BGR2GRAY)
    # img = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
    img = img.astype(np.uint8)

    #get all non zero values
    points = cv2.findNonZero(img)

    avg = np.array([])

    print(points) #avg pixel coordinates
    if (points is not None):
        avg = np.mean(points, axis=0)

        print (avg)
        #print (avg[0][0])
        #print (avg[0][1])
        p1 = int(avg[0][0])
        p2 = int(avg[0][1])
        print ('x coordinate is:')
        print (p1)
        print ('y coordinate is:')
        print (p2)
    
    # point = coord

    # Show distance for a specific point
        p1_o = p1-320
        p2_o = -p2+240
        if (p1_o <= 10 and p1_o >= -10):
            x_in_range = "Yes"
        else:
            x_in_range = "No"

        if (p2_o <= 10 and p2_o >= -10):
            y_in_range = "Yes"
        else:
            y_in_range = "No"


        cv2.circle(color_frame, (p1,p2), 10, (255, 0, 0)) # creates a circle around 'point'of diam 4 and blue colour
        distance = depth_frame[p2, p1]
        distance_origin = depth_frame[320, 240]
        distance_x = depth_frame[0, p1]
        # print ('distance to laser is:')
        # print(distance)
        # cv2.putText(color_frame, "{}mm".format(distance), (400, p2-50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        # cv2.putText(color_frame, "x: {}".format(p1-320), (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        # cv2.putText(color_frame, "{}".format(x_in_range), (170, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        # cv2.putText(color_frame, "y: {}".format(-p2+240), (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        # cv2.putText(color_frame, "{}".format(y_in_range), (170, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.putText(color_frame, "x: {}".format(distance_x), (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)


        cv2.putText(color_frame, "d_o: {}mm".format(distance_origin), (320, 240), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        angle = np.arccos(distance_origin / distance_x)
        angle_d = 180*angle/np.pi
        cv2.putText(color_frame, "{} o".format(angle_d), (320, 300), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        #else:
        #   print ('No point detected')

            
        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)

        write_string = "x: " + str(p1) + " y: " + str(p2) + " d: " + str(distance) + "\n" #String to be written to txt file
        write_string = "x: " + str(p1-320) + " y: " + str(-p2+240) + " d: " + str(distance) + "\n" #String to be written to txt file (0,0 at centre of screen)

        cv2.putText(color_frame, "y: {}".format(-p2+240), (100, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.putText(color_frame, "dx: {}mm".format(distance_x), (200, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        
        ### DISTANCE TO LASER!
        dist_x_h = distance_origin / np.cos(angle)

        # with open('data.txt', 'a') as f:
        # with open('data.txt', 'w') as f:
        #     f.write(write_string)

    
    

    key = cv2.waitKey(100)
    if key == 27:
        break