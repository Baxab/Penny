#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import String, Bool
from controller.msg import ControllerTransition
from tf2_msgs.msg import TFMessage


personID = ""
person_to_track = 0
temp_person_to_track = 0
frame_count = 0
lowest_distance = 4
open_times = 0
cancelled = False

def definition():
    global current_state
    current_state = 1


def deal_with_tracker(tracker_data):
    global person_to_track
    global temp_person_to_track
    global frame_count
    global lowest_distance
    global cancelled
    frame_id = tracker_data.transforms[0].header.frame_id
    if current_state == 1 and frame_id == "openni_depth_frame" and cancelled == False: #TODO
        if tracker_data.transforms[0].child_frame_id[:5] == "torso":
            if frame_count >= 6:
                print "doot", frame_count, person_to_track, lowest_distance
                frame_count = 0
                lowest_distance = 4

                speak_pub.publish("YOU! STOP!")
                speak_pub.publish("pls")
                person_to_track = temp_person_to_track
                print "Cancelling Goal"
                transition = ControllerTransition(states[current_state].__name__, "cancel_goal")
                cancelled = True
                pub.publish(transition)
            else:
                this_distance = tracker_data.transforms[0].transform.translation.x
                if this_distance < lowest_distance:
                    temp_person_to_track = tracker_data.transforms[0].child_frame_id[-1:]
                    lowest_distance = this_distance
                frame_count += 1

def state_sound_alarm(change_state_data):
    return 0
         
def state_navigation(change_state_data):
    print "state_navigation", change_state_data
    if "canceled_goal" in change_state_data.data:
        print("----------------CANCELED GOAL")
        cancelled = False
        return 3
    elif("door" in change_state_data.data):
        print("----------------REACHED DOOR")
        return 2
    elif("True" in change_state_data.data):
        print("----------------REACHED")
        return 1
    else:
        print("----------------NOT REACHED")
        return 0

def state_open_door_detection(change_state_data):
    global open_times
    if change_state_data.data:
	open_times += 1
        if (open_times > 3):
            return 0
        speak_pub.publish("Is anyone there? The door is open!")
        rospy.sleep(5)
        return 2
    else:
        open_times = 0
        return 1
    
def state_follower(change_state_data):
    if change_state_data.data:
        print "Follower Succeded"
        return 4;
    else:
        print "Follower Failed"
        return 0;

def state_face_recognition(change_state_data):
    if change_state_data:
        return 1
    else:
        return 6

def state_face_training(change_state_data):
    if change_state_data:
        return 1
    else:
        return 0
    
def state_card_recognition(change_state_data):
    global personID
    if change_state_data.data != "No Card":
        personID = change_state_data.data
        return 5
    else:
        return 0


# map the inputs to the function blocks
states = {
            0 : state_sound_alarm,
            1 : state_navigation,
            2 : state_open_door_detection,
            3 : state_follower,
            4 : state_face_recognition,
            5 : state_face_training,
            6 : state_card_recognition,
}

# In charge of deciding new state and publishing the update
def callback(change_state_data):
    global current_state
    global personID

    #rospy.loginfo(change_state_data)
	
    current_state = states[current_state](change_state_data)    # decides next state

    print "Moving to state", current_state, "(", states[current_state].__name__, ")"
    if current_state == 0:
        rospy.signal_shutdown("SOUND ALARM")
    elif current_state == 1:
        print "About to start navigation"
        time.sleep(5)
        print "Staring Navigation"
        transition = ControllerTransition(states[current_state].__name__, "")
    elif current_state == 3:
        transition = ControllerTransition(states[current_state].__name__, str(person_to_track))
    elif current_state == 5:
        transition = ControllerTransition(states[current_state].__name__, personID)
        personID = ""
    else:
        transition = ControllerTransition(states[current_state].__name__, "")
            
    pub.publish(transition)
	

if __name__ == '__main__':
    rospy.init_node('controller')

    definition()

    rospy.Subscriber('tf', TFMessage, deal_with_tracker)

    rospy.Subscriber('state_sound_alarm', Bool, callback)
    rospy.Subscriber('state_navigation', String, callback)
    rospy.Subscriber('state_open_door_detection', Bool, callback)
    rospy.Subscriber('state_follower', Bool, callback)
    rospy.Subscriber('state_face_recognition', Bool, callback)
    rospy.Subscriber('state_face_training', Bool, callback)
    rospy.Subscriber('state_card_recognition', String, callback)
    rospy.Subscriber('state_speech', Bool, callback)

    pub = rospy.Publisher('state_machine/control', ControllerTransition, queue_size=10)
    speak_pub = rospy.Publisher('speak', String, queue_size=10)
    
    rospy.spin()
    
    
