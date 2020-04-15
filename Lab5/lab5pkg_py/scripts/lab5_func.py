#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
import math
#from lab5_header import *
PI = np.pi

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.array([[0.       , -1.       , 0.      ,      0.39], \
                  [0.       , 0.       , -1.      ,      0.401], \
                  [1.       , 0.       , 0.      ,      0.2155], \
                  [0.       , 0.       , 0.      ,      1.  ]])
    S = np.array([[ 0.   ,  0.   ,  0.   ,  0.   ,  1.   ,  0.   ],\
                  [ 0.   ,  1.   ,  1.   ,  1.   ,  0.   ,  1.   ],\
                  [ 1.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ],\
                  [ 0.15 , -0.162, -0.162, -0.162,  0.   , -0.162],\
                  [ 0.15 ,  0.   ,  0.   ,  0.   ,  0.162,  0.   ],\
                  [ 0.   , -0.15 ,  0.094,  0.307, -0.26 ,  0.39 ]])




	
	# ==============================================================#
    return M, S


def skew(S):
    return np.array([[0,-S[2],S[1],S[3]],[S[2],0,-S[0],S[4]],[-S[1],S[0],0,S[5]],[0,0,0,0]])

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
    return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
    print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()

    for i in range(6):
        SS = skew(S[:,i])
        T = np.dot(T,expm(SS*theta[i]))
    T = np.dot(T,M)








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

    l01 = 0.152
    l02 = 0.120
    l03 = 0.244
    l04 = 0.093
    l05 = 0.213
    l06 = 0.083
    l07 = 0.083
    l08 = 0.082    
    l09 = 0.0535
    l10 = 0.059   # thickness of aluminum plate is around 0.006

    xgrip = xWgrip + 0.15
    ygrip = yWgrip - 0.15
    zgrip = zWgrip - 0.01
    
    Yaw = yaw_WgripDegree*PI/180.

    xcen = xgrip - l09*np.cos(Yaw)
    ycen = ygrip - l09*np.sin(Yaw)
    zcen = zgrip
    
    d = l02 - l04 + l06
    r = np.sqrt(xcen**2 + ycen**2)
    alpha = math.atan2(ycen,xcen)
    beta = math.atan2(d,np.sqrt(r**2-d**2))

	# theta1
    thetas[0] = alpha - beta      # Default value Need to Change
    
    thetas[4]=-PI/2      # Default value Need to Change

	# theta6
    thetas[5] = thetas[0] + PI/2 - Yaw     # Default value Need to Change
 
    x3end = xcen - l07*np.cos(thetas[0]) + d*np.sin(thetas[0])
    y3end = ycen - l07*np.sin(thetas[0]) - d*np.cos(thetas[0])
    z3end = zcen + l10 +l08
    
    L = np.sqrt(x3end**2 + y3end**2 + (z3end-l01)**2)
    
    thetas[2]= PI-math.acos((l03**2+l05**2-L**2)/(2*l03*l05))      # Default value Need to Change
    
    alpha1 = math.asin((z3end-l01)/L)
    alpha2 = math.acos((l03**2+L**2-l05**2)/(2*l03*L))

    thetas[1]= -alpha1-alpha2     # Default value Need to Change
	
    thetas[3]= -thetas[1] - thetas[2] # Default value Need to Change, need + (0.5*PI) for compensation
	

    print("theta1 to theta6: " + str(thetas) + "\n")

    return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )
