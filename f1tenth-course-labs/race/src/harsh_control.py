#!/usr/bin/env python3
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from dist_finder import desired_distance

# PID Control Params
global kp 
kp = 1.5
#TODO
global kd
kd = 0.5
#TODO
global ki
ki = 1.0
#TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
accumulated_error = 0.0

global vel_input
# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 5.0	#TODO

# Publisher for moving the car. 
# TODO: Use the coorect topic /car_x/offboard/command.
command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

def control(data: pid_input):
	global prev_error

	global kp
	global kd
	global ki
	global correction_term
	global accumulated_error
	global prev_error
	correction_term = 0.0

	
	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	
	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	correction_term = kp*data.pid_error + kd*((data.pid_error - prev_error))
	prev_error = data.pid_error
	accumulated_error = accumulated_error + data.pid_error
	
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
	main_command = 	AckermannDriveStamped()

	# TODO: Make sure the steering value is within bounds [-100,100]
	
	command.steering_angle = command.steering_angle + correction_term

	if command.steering_angle < (-100):
		command.steering_angle = -100
	if command.steering_angle > 100:
		command.steering_angle = 100

	if abs(command.steering_angle) > 0.5:
		if data.pid_vel > 7.0:
			command.steering_angle = 0

	# TODO: Make sure the velocity is within bounds [0,100]
	vel_given = vel_input
	vel_given = (1 - (abs(data.pid_error)/(desired_distance+0.3)))*vel_input
	command.speed = vel_given

	# Move the car autonomously
	main_command.drive = command
	command_pub.publish(main_command)

if __name__ == '__main__':
	rospy.init_node('pid_controller', anonymous=True)
	print("PID Control Node is Listening to error")
	rospy.Subscriber("/error", pid_input, control)
	rospy.spin()