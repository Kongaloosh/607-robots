#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import robot_command
from beginner_tutorials.msg import robot_command_state
import random
import time

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


if __name__ == "__main__":
    x = 0
    y = 0

    while True:
        command = random.randint(0, 1)
        print command
        if command == 0:
            x = 10
        elif command == 1:
            x = -10
        elif command == 2:
            y = 10
        elif command == 3:
            y = -10
        print(command)
        robot_command_client(x, y, command)
        pub.publish(4, command)
        time.sleep(0.2)
    sys.exit(1)