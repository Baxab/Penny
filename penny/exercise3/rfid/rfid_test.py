#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from controller.msg import ControllerTransition
import time
import subprocess

timeout = 5

def read_rfid():
    try:
        output = subprocess.check_output(['/data/private/robot/git/libnfc/utils/nfc-mfultralight', 'r', 'doot'], stderr=subprocess.PIPE)
    except subprocess.CalledProcessError:
        return "No Card"

    return output.split("UID: ")[1][:14]

    
def callback(data):
    if data.state == "state_card_recognition":
        cardValue = "No Card"
        timeRunning = 0
        print "You have " + str(timeout) + " seconds to put your card on the reader"
        startTime = time.time()
        while cardValue == "No Card" and timeRunning < timeout:
            cardValue = read_rfid()
            timeRunning = time.time() - startTime
        print cardValue
        pub.publish(cardValue)

if __name__ == '__main__':
    rospy.init_node('rfid_driver_node')
    rospy.Subscriber('state_machine/control', ControllerTransition, callback)
    pub = rospy.Publisher('state_card_recognition', String, queue_size=100)

    #time.sleep(1)
    rospy.spin()
