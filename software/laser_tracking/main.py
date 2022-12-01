import cv2
import numpy as np

#initialise object
tracker = cv2.VideoCapture(0)

tracker.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # disable auto exposure
tracker.set(cv2.CAP_PROP_EXPOSURE, -9.0)

#pts = []
laser_detector = cv2.createBackgroundSubtractorMOG2(history = 10000, varThreshold = 10000)

while (1):
    # Take each frame
    ret, frame = tracker.read()
    cv2.imshow('original', frame)

##############################################################################################    
    #Detect laser motion
    mask_motion = laser_detector.apply(frame)
    contours, _ = cv2.findContours(mask_motion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        #calculate area and remove big elements
        area = cv2.contourArea(cnt)
        if area > 100 and area < 500:#
            x, y, w, h = cv2.boundingRect(cnt)
            #cv2.drawContours(frame, [cnt], -1, (0,255,255), 2)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,255), 3)

    cv2.imshow('mask_motion', mask_motion)

###############################################################################################
    # Detect laser colour
    
    # Convert BGR to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
    
    result = cv2.bitwise_and(frame, frame, mask=full_mask)
    cv2.imshow('result',result)
    
    #blurring test
    kernel = np.ones((15,15), np.float32)/225
    
    smoothed = cv2.filter2D(result, -1, kernel)
    blur = cv2.GaussianBlur(result, (15,15), 0)
    median = cv2.medianBlur(result,15)
    bilateral = cv2.bilateralFilter(result, 15, 75, 75)
    #cv2.imshow('smoothed',smoothed)

    #Threshold the HSV image to get only red colors

    mask_colour = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
    cv2.imshow('mask_colour', mask_colour)

    red_filter = cv2.bitwise_and(frame, frame, mask = mask_colour)
    cv2.imshow('red_filter', red_filter)

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask_colour)

########################################################################################
    #the circle to prove tracking laser
    #cv2.circle(red_filter, maxLoc, 20, (10, 100, 20), 2, cv2.LINE_AA)
    #cv2.imshow('Track Laser', red_filter)
    cv2.circle(frame, maxLoc, 20, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow('Track Laser', frame)

    #print(maxLoc)
    #cv2.imshow('mask', full_mask)
    #cv2.imshow('result', tracker)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

tracker.release()
cv2.destroyAllWindows()