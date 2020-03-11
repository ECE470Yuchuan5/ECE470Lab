#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6,6))
    M = np.array([[0.       , -1.       , 0.      ,      540], \
                  [0.       , 0.       , -1.      ,      192], \
                  [1.       , 0.       , 0.      ,      152], \
                  [0.       , 0.       , 0.      ,      1.  ]])
    p1 = np.array([[0.],[0.],[0.]])
    p2 = np.array([[0.],[120],[152]])
    p3 = np.array([[244.],[120.],[152]])
    p4 = np.array([[457],[27],[152]])
    p5 = np.array([[457],[110],[152]])
    p6 = np.array([[540],[110],[152]])
    w1 = np.array([[0.],[0.],[1.]])
    w2 = np.array([[0.],[1],[0]])
    w3 = np.array([[0.],[1],[0.]])
    w4 = np.array([[0.],[1],[0.]])
    w5 = np.array([[1.],[0.],[0.]])
    w6 = np.array([[0.],[1.],[0.]])
    v1 = -np.cross(np.transpose(w1),np.transpose(p1))
    v2 = -np.cross(np.transpose(w2),np.transpose(p2))
    v3 = -np.cross(np.transpose(w3),np.transpose(p3))
    v4 = -np.cross(np.transpose(w4),np.transpose(p4))
    v5 = -np.cross(np.transpose(w5),np.transpose(p5))
    v6 = -np.cross(np.transpose(w6),np.transpose(p6))
    S1 = np.r_[w1,np.transpose(v1)]
    print(S1)
    S2 = np.r_[w2,np.transpose(v2)]
    S3 = np.r_[w3,np.transpose(v3)]
    S4 = np.r_[w4,np.transpose(v4)]
    S5 = np.r_[w5,np.transpose(v5)]
    S6 = np.r_[w6,np.transpose(v6)]
    S = np.c_[S1,S2,S3,S4,S5,S6]



	
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



