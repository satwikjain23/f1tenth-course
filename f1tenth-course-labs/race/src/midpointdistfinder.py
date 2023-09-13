#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan

from race.msg import pid_input
#(LEVINE-)
# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 0.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.(1.5)(0.5)
#desired_distance = 0.6	# distance from the wall (in m). (defaults to right wall)(0.9)
vel = 2.5	# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
c=0.6


# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('/err', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	index = (angle)*(len(data.ranges)/360)
	index = int(index)
	
	#print(data.angle_min)
	#print(data.angle_max)
	#print(data.angle_increment)
	
	return data.ranges[index]



def callback(data):
	global forward_projection

	theta = 135 # you need to try different values for theta(70)
	a = getRange(data,theta) # obtain the ray distance for theta
	e = getRange(data,180)
	b = getRange(data,90)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	d = getRange(data,270)
	f= getRange(data,80)
	g= getRange(data,160)
	swing = math.radians(45)

	
	
	
     
	#if c>7:
	#	if (a>1.2):
	#		a=0.9
	#	if (b >2):
	#		b=1.3
	# 

	
	
	
	 
 
	## Your code goes here to determine the error as per the alrorithm 
	# Compute Alpha, AB, and CD..and finally the error.
	# TODO: implement
	Alpha = math.atan((a*(math.cos(swing)) - b)/(a*math.sin(swing)))
	AB = b*math.cos(Alpha)
	CD = AB + (forward_projection*(math.sin(Alpha)))
	desired_distance=((((b*math.cos(Alpha))+(d*math.cos(Alpha))))/2)
	#print(desired_distance)/(-0.2)
	#-0.3berlin
	print("Alpha=",Alpha)
	print("180=",d*math.cos(Alpha))
	print("0=",b*math.cos(Alpha))
	#print("f=",f)

	print("b+d=",((b*math.cos(Alpha))+(d*math.cos(Alpha))))
	print("desired distance=",desired_distance)
	#print("e=",e)
	
	error = CD-desired_distance
	#print("error==",error)
	print("--------------------------")
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.front_distance=e
	msg.front_right=f
	msg.front_left=g
	msg.pid_vel=1

	'''
	if(e<1):
		msg.pid_vel = 0.2
	else:
		msg.pid_vel=0.5	# velocity error can also be sent.
	'''
	pub.publish(msg)
	
	


if __name__ == '__main__':
	c=0
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.spin()