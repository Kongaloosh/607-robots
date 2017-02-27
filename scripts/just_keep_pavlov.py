__author__ = 'kongaloosh'


#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import robot_command
from beginner_tutorials.msg import robot_command_state
import random
import time
from beginner_tutorials.msg import servo_state, verifier, gvf

__author__ = 'kongaloosh'

pub = rospy.Publisher('robot_obeservation', robot_command_state, queue_size=10)     # publishing to
rospy.init_node('robot_command_talker', anonymous=True)                             # initializes node with name


def robot_command_client(x, y, command):
    rospy.wait_for_service('robot_controller')
    try:
        command_service = rospy.ServiceProxy('robot_controller', robot_command)
        resp1 = command_service(x, y, command)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def pavlovs_bell(data):
    if data.prediction_normalized > threshold:
        x = -2,
        y = -2
        robot_command_client(x, y, 3)
        pub.publish(4, 3)
    else:
        x = 2
        y = 0
        robot_command_client(x,y, 4)



if __name__ == "__main__":
    rospy.init_node('robot_command_talker', anonymous=True)                             # initializes node with name
    pub = rospy.Publisher('robot_obeservation', robot_command_state, queue_size=10)     # publishing to
    rospy.Subscriber('position_predictior', gvf, pavlovs_bell)  # subscribes to chatter and calls the callback

    rospy.spin()  # keeps python from exiting until this node is stopped

    threshold = 4
    x = 0
    y = 0
    command = 0
    while True:
        if command == 1:
            command = 0
        else:
            command = 1

        if command == 0:
            x = 2
        elif command == 1:
            x = -2
        elif command == 2:
            y = 2
        elif command == 3:
            y = -2

        robot_command_client(x, y, command)
        pub.publish(4, command)
        time.sleep(3)
    sys.exit(1)
