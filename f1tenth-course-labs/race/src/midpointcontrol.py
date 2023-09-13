#!/usr/bin/env python3
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

#levine(kp-1.0/0.7,kd-0.8)
# PID Control Params
global kp 
kp = 2.0
#TODO(1.4)(1.0)(2)
global kd
kd = 0.8
#TODO(0.4)(0.8)
global ki
ki = 0.0
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
errorsum=0

 
# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
#vel_input = data.pid_vel	#TODO(1)

# Publisher for moving the car. 
# TODO: Use the coorect topic /car_x/offboard/command.
command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

def control(data: pid_input):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	global errorsum
	angle=0.0

	vel_input = data.pid_vel
	
	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	
	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	
	errorsum=data.pid_error
	angle = kp*data.pid_error + kd*((prev_error - data.pid_error ))+ ki*errorsum
	print("error=",errorsum)
	print("angle=",angle)
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
	
	
	#print("-----------------")



	#LEVINE
	'''
	if ((angle < (-0.15))and(data.front_distance<0.6)):
		angle = -angle+1
		print("inverse")
	elif(((angle > (0.15))and(data.front_distance<0.5))):
		angle = -angle-0.2
		print("reverse")
	
    '''

	#BERLIN
	'''
	if(data.front_right<1.2):
		angle=-0.6
	elif(data.front_left<1.2):
		angle=0.6

    '''

	'''
	if(angle)>0.6:
		angle=0.07
	elif(angle<-0.6):
		angle=-0.07
	print("extreme case=",angle)
	'''
	# TODO: Make sure the steering value is within bounds [-100,100]
	an=command.steering_angle-angle
	print("an",an)
	print("------------------------")
	
	command.steering_angle = an

	

	# TODO: Make sure the velocity is within bounds [0,100]
	command.speed = vel_input
	command.steering_angle_velocity = 0.0
	command.acceleration = 0.0
    

	new= AckermannDriveStamped()
	new.drive=command
	# Move the car autonomously
	command_pub.publish(new)
	prev_error = data.pid_error
	

if __name__ == '__main__':
	
	#kp = input("Enter Kp Value: ")
	#kd = input("Enter Kd Value: ")
	#ki = input("Enter Ki Value: ")
	#vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
	print("PID Control Node is Listening to error")
	rospy.Subscriber("/err", pid_input, control)
	rospy.spin()