#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point

# Params for camera calibration
theta = -0.09
beta = 730.0
tx=-0.297435628054
ty=-0.0763807206584

# Params for camera
w = 640
h = 480

# Params for block
block_height = 0.015

# Place holders for block world positions
block_green_world = None
block_yellow_world = None

# Function that converts image coord to world coord
def IMG2W(r,c):
    global theta
    global beta
    global tx
    global ty

    ################################ Your Code Start Here ################################
    # Given theta, beta, tx, ty, calculate the world coordinate of r,c namely xw, yw
    Or = r - h
    Oc = c - w
    xr = Or/beta
    yr = Oc/beta
    xc = xr*np.cos(theta) - yr*np.sin(theta)
    yc = yr*np.cos(theta) + xr*np.sin(theta)
    xw = xc + tx
    yw = yc + ty

    # check function with nikhils input output values

    ################################# Your Code End Here #################################
    return xw, yw

def block_green_pixel_callback(data):

    global block_green_world

    xw, yw = IMG2W(data.x, data.y)

    block_green_world = Point()
    block_green_world.x = xw
    block_green_world.y = yw
    block_green_world.z = block_height

def block_yellow_pixel_callback(data):

    global block_yellow_world

    xw, yw = IMG2W(data.x, data.y)

    block_yellow_world = Point()
    block_yellow_world.x = xw
    block_yellow_world.y = yw
    block_yellow_world.z = block_height

if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node('lab5_coordinate_converter')

    # Publish at 10 Hz
    rate = rospy.Rate(10)

    # Define block world position publishers
    pub_block_green_world = rospy.Publisher('block_green/world', Point, queue_size=10)

    pub_block_yellow_world = rospy.Publisher('block_yellow/world', Point, queue_size=10)

    # Define block pixel coordinates subscribers
    sub_block_green_pixel = rospy.Subscriber('block_green/pixel', Point, block_green_pixel_callback)

    sub_block_yellow_pixel = rospy.Subscriber('block_yellow/pixel', Point, block_yellow_pixel_callback)

    while not rospy.is_shutdown():
        if block_green_world is not None:
            pub_block_green_world.publish(block_green_world)
        if block_yellow_world is not None:
            pub_block_yellow_world.publish(block_yellow_world)
        rate.sleep()
