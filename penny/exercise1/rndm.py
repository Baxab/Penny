#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time
import random


speed = 0.3    # Setting speed of the Robot
max_range = 3   # Maximum Range that the laser can detect
angle = 1       # Angle of rotation
t1 = 0
t2 = 0


# Function to calculate the mean obtained from the array of LaserScan values 
def mean(sensor_values):               
	total = 0
	i = 0
	for data in sensor_values:
	        if data != 0:
	            total += data
	            i += 1;
	result = total / i
	return result
	
# Function to decide the angle of rotation
def turn(sensor_data):
	right_average = mean(sensor_data[:200]) # The array of the LaserScan values has been split into three sectors. The first 200 values
	left_average = mean(sensor_data[300:])  # represent the right sector. The last 200 values represent the left sector. The direction 
	result = left_average - right_average   # of rotation is determined only by the average of the values in the left and right sector.
	print "Right: " + str(right_average)    # The robot rotates in the direction of the greater average distance.
	print "Left: " + str(left_average)
	print "Result" + str(result)
	if result < 0:
		result = -(angle)
	else:
		result = angle
	return result

pastTime = 0   
def callback(sensor_data):
	currentTime = time.time()
	
	if (currentTime - pastTime) > 0.1:      
		      

		data_sns = []
		for num, data in enumerate(sensor_data.ranges):  # Setting zero for values that could not be a number,setting maximum range 
			if isnan(data):                          # for values that could be greater than maximum range and putting it into
				data_sns.append(0)               # a new array.
			elif isinf(data):
				data_sns.append(max_range)
			else:
				data_sns.append(data)

		
		base_data = Twist()                              # Setting the linear motion to the required speed.
		base_data.linear.x = speed	
	
		arc_start = 125                                  # The values for the start and end of the arc for the vision in the front 
		arc_end = 375                                    # and minimum detection distance have been set after experimentation.
		detection_distance = 0.5
		
		for i in range(len(data_sns[arc_start:arc_end]) - 5):         # Detect an object right in front of the robot and turn.
		        if data_sns[arc_start+i] < detection_distance:
		                if data_sns[arc_start+i+1] < detection_distance:
		                        if data_sns[arc_start+i+2] < detection_distance:
		                                if data_sns[arc_start+i+3] < detection_distance:
						        if data_sns[arc_start+i+4] < detection_distance:
						                if data_sns[arc_start+i+5] < detection_distance:
						                        base_data.linear.x = 0
									base_data.angular.z = turn(data_sns)
									break
									
	        if (time.time() - t1) > 20:
	                diff = abs(mean(data_sns[:200])- mean(data_sns[300:]))
	                if diff < 0.1:
	                       i = random.randint(0,1000)
	                       if ( mean(data_sns[:200]) > 1 and mean(data_sns[300:]) > 1):
	                            if (i%2 == 0):
	                              base_data.linear.x = 0
	                              base_data.angular.z = 22.5
	                              print "largerandom"
	                              
	                            else:
	                              base_data.linear.x = 0
	                              base_data.angular.z = -22.5
	                              print "largerandom" 
	                              
	                       elif ( mean(data_sns[:200]) < 0.6 and mean(data_sns[300:]) < 0.6):
	                            if (i%2 == 0):
	                              base_data.linear.x = -0.3
	                              base_data.angular.z = 22.5
	                              print "smallrandom"
	                              
	                            else:
	                              base_data.linear.x = -0.3
	                              base_data.angular.z = -22.5 
	                              print "smallrandom"
	                        
	                              
	                        

		pub.publish(base_data)              #Publish data
		global pastTime
	        pastTime = currentTime

     

if __name__ == '__main__':
	rospy.init_node('reactive_mover_node')
	t1 = time.time()
	rospy.Subscriber('base_scan', LaserScan, callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	rospy.spin()

	
