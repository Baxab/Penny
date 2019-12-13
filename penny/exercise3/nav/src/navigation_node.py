#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from controller.msg import ControllerTransition
    
def definition():
    global STATE_NAME
    global CANCEL_GOAL
    global TIME_TO_SOUND_ALARM
    global checkpoints	
    global current_point
    global goalReached
    global doors
    STATE_NAME = 'state_navigation'
    CANCEL_GOAL = 'cancel_goal'
    TIME_TO_SOUND_ALARM = 180
    checkpoints = [[8.3713312149,0.806817531586,0.979428482338,0.20179159543],[5.00263547897,2.33208847046,0.55745770379,0.830205341157],[0.0087910592556,4.98708152771,0.56425779093,0.825598658777],[-2.46205329895,5.36922025681,0.553883609613,0.832594106995],[-5.46993255615,6.29303073883,-0.191575278382,0.981477922682],[2.1274920466,0.292680025101,-0.203367899069,0.979102393843],[-3.40268158913,-2.98744797707,0.978951784423,0.204091655328],[-5.71687503629,-8.07002258301,0.984125046578,0.177476456744],[-3.97589254379,-14.5152082443,-0.165426215499,0.986222169303],[0.294730186462,-19.0036392212,0.989500613705,0.144528666626],[-8.74712944031,-16.6055648804,-0.812189388978,0.583393860468],[-13.6802968979,-14.2574901581,-0.822767540636,0.568378020401]]
    doors = [1,2,3,5,6,7,8,10,11]
    current_point = 0


def callback(change_state_data):
    global STATE_NAME
    global CANCEL_GOAL
    global current_point
    global doors
    
    if(STATE_NAME in change_state_data.state):
        if(CANCEL_GOAL in change_state_data.data):
            cancel_navigation()
            pub.publish("canceled_goal")
        else:
            goalReached = moveToGoal(checkpoints[current_point][0], checkpoints[current_point][1], checkpoints[current_point][2], checkpoints[current_point][3])
            if(current_point in doors):
                isDoor = "_door"
            else:
                isDoor = ""
            update_checkpoint(goalReached)
            pub.publish(str(goalReached)+isDoor)

def moveToGoal(xGoal, yGoal, zOrientGoal, wOrientGoal):
    global TIME_TO_SOUND_ALARM
    global ac
    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
     #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = zOrientGoal
    goal.target_pose.pose.orientation.w = wOrientGoal

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration.from_sec(TIME_TO_SOUND_ALARM))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False
            
def cancel_navigation():
    global ac
    ac.cancel_all_goals()
    update_checkpoint(False)
        
def update_checkpoint(goalReached):
    global current_point
    global checkpoints
    if goalReached:
        if current_point < (len(checkpoints)-1):
            current_point += 1
        else:
            current_point = 0
        return True
    else:
        return False

   

               
if __name__ == '__main__':
    rospy.init_node('map_navigation')

    definition()

    rospy.Subscriber('state_machine/control', ControllerTransition, callback)

    pub = rospy.Publisher('state_navigation', String, queue_size=10)
    
    rospy.spin()
    
    
    
