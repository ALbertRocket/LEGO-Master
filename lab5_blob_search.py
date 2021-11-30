#!/usr/bin/env python

import cv2
import numpy as np
from geometry_msgs.msg import Point
from lab5_coordinate_converter import *

def blob_search(image_raw, color):

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    
    # Edit Below
    ##################################
    # Replace these with your values for each color
    bluelower = (110, 50, 50)   # Blue lower
    blueupper = (130, 255, 255) # Blue upper
    greenlower = (45, 130, 110)
    greenupper = (65, 255, 255)
    yellowlower = (25, 50, 50)
    yellowupper = (35, 255, 255)

    if (color == "yellow"):
        lower = yellowlower
        upper = yellowupper
        min = 0.75
        max = 0.85
        minA=100
        maxA=700
    elif (color == "green"):
        lower = greenlower
        upper = greenupper
        min = 0.75
        max = 0.85
        minA=100
        maxA=700
    elif (color == "cylinder"):
        lower = greenlower
        upper= greenupper
        min= 0.8
        max= 1.1
        minA=100
        maxA=700
    elif (color == "rectangle"):
        lower = yellowlower
        upper = yellowupper
        min = 0.6
        max = 0.85
        minA= 700
        maxA= 1500

    # Edit Above
    ##################################

    mask_image =  cv2.inRange(hsv_image, lower, upper)

    # Edit Below
    ##################################
    # Setup SimpleBlobDetector parameters by editing the following code:
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = minA
    params.maxArea = maxA

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = min
    params.maxCircularity = max

    # Filter by Inertia
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # Edit Above
    ##################################

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # Draw the keypoints on the detected block
    #im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, image_raw)
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    xw_yw = []

    if(num_blobs == 0):
        #print("No block found!")
        asdasdasd=1
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))

    #cv2.namedWindow("Press Enter to Continue")
    #cv2.imshow("Press Enter to Continue", im_with_keypoints)
    
    #cv2.namedWindow("Camera View")
    #cv2.imshow("Camera View", image_raw)
    # cv2.namedWindow("Mask View")
    #cv2.imshow("Mask View", mask_image)
    # cv2.namedWindow("Keypoint View")
    # cv2.imshow("Keypoint View", im_with_keypoints) 
       
    return xw_yw
 
