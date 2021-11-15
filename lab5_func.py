#!/usr/bin/env python
import numpy as np
import math
from math import asin as asin
from math import acos as acos
from math import atan2 as atan2
from math import atan as atan
from math import sin as sin
from math import cos as cos
from scipy.linalg import expm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    global S
    global M
    M = np.eye(4)
    S = np.zeros((6,6))
    S1 = [0, 0, 1, 0.15,0.15,0]
    S2 = [0, 1, 0, -0.162,0,-0.15]
    S3 = [0, 1, 0, -0.162, 0 ,0.094]
    S4 = [0, 1, 0, -0.162, 0,0.307]
    S5 = [1, 0, 0, 0, 0.162, -0.260]
    S6 = [0, 1, 0, -0.162,0,0.39]
    S = np.matrix([S1,S2,S3,S4,S5,S6])
    M = np.matrix([[0, -1, 0,0.39],[0, 0, -1, 0.401],[1, 0, 0, 0.2155],[0,0,0,1]])





	# ==============================================================#
    return M, S
    
def Screw_matrix(Svector):
    Smatrix = np.matrix([[0,-Svector[:,2], Svector[:,1], Svector[:,3]],[Svector[:,2], 0, -Svector[:,0], Svector[:,4]],[-Svector[:,1],Svector[:,0], 0, Svector[:,5]],[0, 0, 0, 0]])

    return Smatrix

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
    return_value = [None, None, None, None, None, None]

    print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)
    M, S = Get_MS()
    s1 = S[0]
	#print(s1)
    s2 = S[1]
	#print(s2)
    s3 = S[2]
	#print(s3)
    s4 = S[3]
	#print(s4)
    s5 = S[4]
	#print(s5)
    s6 = S[5]
	#print(s6)
    s1_m = Screw_matrix(s1)
    s2_m = Screw_matrix(s2)
    s3_m = Screw_matrix(s3)
    s4_m = Screw_matrix(s4)
    s5_m = Screw_matrix(s5)
    s6_m = Screw_matrix(s6)

    T1 = np.matmul(expm(s1_m*theta1),expm(s2_m*theta2))
    T2 = np.matmul(T1,expm(s3_m*theta3))
    T3 = np.matmul(T2,expm(s4_m*theta4))
    T4 = np.matmul(T3,expm(s5_m*theta5))
    T5 = np.matmul(T4,expm(s6_m*theta6))
    T = np.matmul(T5,M)
	#print(T)	


	# ==============================================================#

    print(str(T) + "\n")

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    l1 = 0.152
    l2 = 0.120
    l3 = 0.244
    l4 = 0.093
    l5 = 0.213
    l6 = 0.083
    l7 = 0.083
    l8 = 0.082
    l9 = 0.0535
    l10 = 0.059   # thickness of aluminum plate is around 0.01

    xgrip = xWgrip + 0.15
    ygrip = yWgrip - 0.15
    zgrip = zWgrip - 0.01

    xcen = xgrip - l9*np.cos(yaw_WgripDegree*math.pi/180)
    ycen = ygrip - l9*np.sin(yaw_WgripDegree*math.pi/180)
    zcen = zgrip

    beta = math.atan2(ycen,xcen)
    alpha = asin((l2-l4+l6)/(xcen**2+ycen**2)**0.5)
    theta1 = beta - alpha

    theta6 = math.pi/2-yaw_WgripDegree*math.pi/180+theta1

    x3end = (((xcen**2+ycen**2)-(l2-l4+l6)**2)**0.5-l7)*cos(theta1)
    y3end = (((xcen**2+ycen**2)-(l2-l4+l6)**2)**0.5-l7)*sin(theta1)
    z3end = zcen + l10 + l8

    c = (x3end**2+y3end**2+(z3end-l1)**2)**0.5
    a = l5
    b = l3

    theta2 = -np.arccos((b**2+c**2-a**2)/(2*b*c)) - np.arctan2(z3end-l1,(x3end**2+y3end**2)**0.5)
    theta3 = math.pi - np.arccos((a**2+b**2-c**2)/(2*a*b))
    theta4 = -(theta3 + theta2)
    theta5 = -math.pi/2
    thetas = [theta1,theta2,theta3,theta4,theta5,theta6]
    print("theta1 to theta6: " + str(thetas) + "\n")

    return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
                  float(thetas[3]), float(thetas[4]), float(thetas[5]) )
