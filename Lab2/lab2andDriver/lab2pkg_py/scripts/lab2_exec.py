#!/usr/bin/env python

import copy
import time
import rospy
import numpy as np
from lab2_header import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = np.radians([178.74, -80.56, 92.68, -116.68, -89.32, 10.4])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)


############## Your Code Start Here ##############

# Hanoi tower location 1

Q11 = np.radians([168.32, -59.96, 103.88, -135.34, -89.05, 0])
Q12 = np.radians([168.32, -54.99, 105.43, -141.87, -89.05, 0])
Q13 = np.radians([168.32, -48.53, 106.13, -149.02, -89.05, 0])

Q21 = np.radians([178.75, -58.15, 100.53, -133.92, -89.33, 10.41])
Q22 = np.radians([178.75, -53.05, 102.14, -140.64, -89.33, 10.42])
Q23 = np.radians([178.75, -46.05, 102.76, -148.26, -89.33, 10.43])

Q31 = np.radians([188.16, -53.85, 93.30, -131.07, -89.59, 19.82])
Q32 = np.radians([188.16, -49.35, 94.77, -137.03, -89.59, 19.83])
Q33 = np.radians([188.16, -42.79, 95.35, -144.18, -89.59, 19.84])

Qn = np.radians([178.74, -80.56, 92.68, -116.68, -89.32, 10.4])

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]

############### Your Code End Here ###############


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""


def gripper_callback(msg):
    global digital_in_0
    global analog_in_0

    digital_in_0 = msg.DIGIN
    analog_in_0 = msg.AIN0



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0   
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
	           end_loc, end_height):
	global Q
    global Qn
    error = 0
    global digital_in_0

    move_arm(pub_cmd, loop_rate, Qn, 2.0, 2.0)
    
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 2.0, 2.0)

    gripper(pub_cmd, loop_rate, suction_on)
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)

    if(not(digital_in_0)):
        return 1

    move_arm(pub_cmd, loop_rate, Qn, 2.0, 2.0)

    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 2.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_off)

	return error

############### Your Code End Here ###############


def main():

	global home
	global Q
	global SPIN_RATE

	# Initialize ROS node
	rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)

	############## Your Code Start Here ##############
	# TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

	sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_input_callback)

    gripper_in = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

	############### Your Code End Here ###############


	############## Your Code Start Here ##############
	# TODO: modify the code below so that program can get user input

	input_done = 0
	loop_count = 0

	while(not input_done):
        start = int(raw_input("Enter starting position <either 1, 2 or 3>: "))
        print("You entered " + str(start) + "\n")

        end = int(raw_input("Enter ending position <either 1, 2 or 3>: "))
        print("You entered " + str(end) + "\n")
        

        if(start==end):
            print("The starting position is the same as the ending position")
        elif((start<1)or(start>3)or(end<1)or(ending>3)):
            print("Invalid input")
        else:
            input_done = 1



	############### Your Code End Here ###############

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	rospy.loginfo("Sending Goals ...")

	loop_rate = rospy.Rate(SPIN_RATE)

	############## Your Code Start Here ##############
	# TODO: modify the code so that UR3 can move tower accordingly from user input

    if(move_block(pub_command, loop_rate, start-1,0,end-1,2)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1
    if(move_block(pub_command, loop_rate, start-1,1,5-start-end,2)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1
    if(move_block(pub_command, loop_rate, end-1,2,5-start-end,1)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1
    if(move_block(pub_command, loop_rate, start-1,2,end-1,2)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1
    if(move_block(pub_command, loop_rate, 5-start-end,1,start-1,2)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1
    if(move_block(pub_command, loop_rate, 5-start-end,2,end-1,1)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1
    if(move_block(pub_command, loop_rate, start-1,2,end-1,0)):
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)
        return 1

    move_arm(pub_command, loop_rate, Qn, 2.0, 2.0)

	############### Your Code End Here ###############



if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass


	






