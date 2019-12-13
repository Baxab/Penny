#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from controller.msg import ControllerTransition
import time
import subprocess
import pygame

security_cards = ["042c89622e4b80"]
keep_going = True

def callback(data):
    global keep_going
    if data.state == "state_alarm":
        pygame.mixer.music.load("alarm.mp3")
        while keep_going:
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy() and keep_going:
                read_rfid()
         
        pygame.mixer.music.stop()
        pub.publish(True)
        keep_going = True


def read_rfid():
    global keep_going
    card_id = "No Card"
    try:
        output = subprocess.check_output(['/data/private/robot/git/libnfc/utils/nfc-mfultralight', 'r', 'doot'], stderr=subprocess.PIPE)
        card_id = output.split("UID: ")[1][:14]
    except subprocess.CalledProcessError:
        card_id = "No Card"

    if card_id in security_cards:
        keep_going = False


if __name__ == '__main__':
    rospy.init_node('alarm_driver_node')
    rospy.Subscriber('state_machine/control', ControllerTransition, callback)
    pub = rospy.Publisher('state_alarm', Bool, queue_size=100)

    pygame.mixer.init()

    rospy.spin()
