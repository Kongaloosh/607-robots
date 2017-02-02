#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import robot_command
import random

__author__ = 'kongaloosh'

def robot_command_client(x, y):
    rospy.wait_for_service('robot_controller')
    try:
        command_service = rospy.ServiceProxy('robot_controller', robot_command)
        resp1 = command_service(x, y)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    x = 512
    y = 512

    while True:
        command = random.randint(4)
        if command == 1:
            x += 30
        elif command == 2:
            x -= 30
        elif command == 3:
            y += 30
        elif command == 4:
            y -= 30

        robot_command_client(x, y)

    sys.exit(1)
