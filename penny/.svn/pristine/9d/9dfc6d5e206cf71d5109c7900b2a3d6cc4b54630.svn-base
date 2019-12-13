#! /usr/bin/env python

import rospy
from face_recognition.msg import FRClientGoal
from face_recognition.msg import FaceRecognitionActionResult
from controller.msg import ControllerTransition
from std_msgs.msg import String, Bool
import time

recognition_timeout = 5
adding_timeout = 30
face_state = -1

def wait_until_complete(timeout = 100):
    start_time = time.time()
    time_taken = 0
    while face_state != -1 and time_taken < timeout:
        time_taken = time.time() - start_time

    if time_taken >= timeout:
        return False
    else:
        return True

def add_person(name):
    result = add_face_images(name)
    if result:
        train()
    return result

def recognise_once():
    global face_state
    face_state = 0
    data = FRClientGoal(face_state, "")
    pub.publish(data)
    print "Recongising once"
    result = wait_until_complete(timeout = recognition_timeout)
    if not result:
        stop = FRClientGoal(6, "") # This might not work, otherwise find a better way of stopping the recognition
        pub.publish(stop)
    return result

def recognise_continuous():
    global face_state
    face_state = 1
    data = FRClientGoal(face_state, "")
    pub.publish(data)
    print "Recongising continous"
    return wait_until_complete()

def add_face_images(name):
    global face_state
    face_state = 2
    data = FRClientGoal(face_state, name)
    pub.publish(data)
    print "Adding face images"
    return wait_until_complete(timeout = adding_timeout)

def train():
    global face_state
    face_state = 3
    data = FRClientGoal(face_state, "")
    pub.publish(data)
    print "Training"
    return wait_until_complete()

def exit():
    global face_state
    face_state = 4
    data = FRClientGoal(face_state, "")
    pub.publish(data)
    print "Exiting"
    return wait_until_complete()

def recognition_callback(data):
    global face_state
    names = data.result.names
    if face_state == 0 or face_state == 1:
        print "Recognised: " + names[0]
    elif face_state == 2:
        print "Finish adding faces for: " + names[0]
    elif face_state == 3:
        print "Finished training"
    elif face_state == 4:
        print "Finished exiting"
    else:
        print "SHIT"
    face_state = -1

def controller_callback(data):
    if data.state == "state_face_recognition":
        result = recognise_once()
        recognition_pub.publish(result)
    elif data.state == "state_face_training":
        result = add_person(data.data)
        training_pub.publish(result)

if __name__ == '__main__':
    rospy.init_node('face_driver_node')

    rospy.Subscriber('face_recognition/result', FaceRecognitionActionResult, recognition_callback)
    pub = rospy.Publisher('fr_order', FRClientGoal, queue_size=100)

    rospy.Subscriber('state_machine/control', ControllerTransition, controller_callback)
    recognition_pub = rospy.Publisher('state_face_recognition', Bool, queue_size=100)
    training_pub = rospy.Publisher('state_face_training', Bool, queue_size=100)

    rospy.spin()
