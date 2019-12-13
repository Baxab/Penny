#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time


speed = 0.3     # Setting speed of the Robot
max_range = 3.8   # Maximum Range that the laser can detect
angle = 0.8       # Angle of rotation
pastTime = 0    # Past time for calculating time intervals
pastTime2 = 0
currentTime2 = 0

detection_arc_start = 125               # The values for the start and end of the arc for the vision in the front 
detection_arc_end = 375                 # and minimum detection distance have been set after experimentation.
detection_distance = 0.5                # Distance at which an object will be detected
object_size = 5                         # The minimum size of an object to be seen by the detector


count = 0
iii = 0
sign = 0
jjj = 0

right_avg_arr = [0,0,0,0,0,0,0,0,0,0]
left_avg_arr = [0,0,0,0,0,0,0,0,0,0]



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
    right_average = mean(sensor_data[60:200]) # The array of the LaserScan values has been split into three sectors. The first 200 values
    left_average = mean(sensor_data[300:440])  # represent the right sector. The last 200 values represent the left sector. The direction 
    result = left_average - right_average   # of rotation is determined only by the average of the values in the left and right sector.
    #print "Right: " + str(right_average)    # The robot rotates in the direction of the greater average distance.
    #print "Left: " + str(left_average)
    #print "Result" + str(result)
    if result < 0:
        result = -(angle)
    else:
        result = angle
    return result


def callback(sensor_data):
    currentTime = time.time()
    if (currentTime - pastTime) > 0.1: 
	
	print "##################################"     
	#print currentTime-pastTime
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
    
        for i in range(len(data_sns[detection_arc_start:detection_arc_end]) - object_size):# Detect an object right in front of the robot and turn.
            object_count = 0
            for j in range(object_size):
                if(data_sns[detection_arc_start+i+j]) < detection_distance:
                    object_count += 1
            if object_count == object_size:
                base_data.linear.x = 0.1
                turn_value = turn(data_sns)
                base_data.angular.z = turn_value
                break


	global currentTime2
	global count 
	global iii
	global sign
	global jjj	
	#right_avg = mean(data_sns[25:200])
	#left_avg = mean(data_sns[300:475])
	right_avg = mean(data_sns[80:200])
	left_avg = mean(data_sns[300:420])
	#right_avg_arr[count] = right_avg
	#left_avg_arr[count] =  left_avg
	#if iii >= 9:
		#tot_right = 0
	        #tot_left = 0
	    
	        #for x in right_avg_arr:
	        #	tot_right = tot_right + x
		#res_right = tot_right/len(right_avg_arr)
	
		#for y in left_avg_arr:
		#	tot_left = tot_left + y
      		#res_left = tot_left/len(left_avg_arr)
	if sign == 0 and (right_avg > 3.8 or left_avg > 3.8):
		print "SIGN   0"
		diff_means = right_avg - left_avg
		#print diff_means    
		if (diff_means >= 0):
			base_data.angular.z = -0.5
			sign = -1
	    	else:
	        	base_data.angular.z = 0.5
			sign = 1
			jjj = jjj+1
		currentTime2 = time.time()
		if (currentTime2 - pastTime2) > 6:
			#if sign != 0 and (right_avg > 3.8 or left_avg > 3.8):
			print "GIRO ANCORA ================================))))))))))))))))))))))))"
				
				#jjj = jjj+1
				#print jjj
			for z in range(0,10):
				base_data.angular.z = 15*sign
				print "CACCACACCACACCACACCACACCACACCACACCACACCACACCACACCA"
				#pub.publish(base_data)
				#if jjj >= 4:
				#	print "j   RESET"
				#	sign = 0
				#	jjj = 0
			global pastTime2
    			pastTime2 = currentTime2
		else:
			print "to mare omo"
			print currentTime - pastTime2
		sign=0 
		
		
		
	   
			#if count == 9:
			#	count = 0
			#else:
		    	#	count = count+1

		#iii = iii+1
	
	global pastTime
        pastTime = currentTime
        pub.publish(base_data)              #Publish data
        
	

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()

    
