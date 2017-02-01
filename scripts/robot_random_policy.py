#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import robot_command

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

    print("ROBOT ARCADE: \nTo use this, press q or w to move the first servo and o or p to move the second servo."+
          "\npress b to break and exit.")
    while True:
        command = getKey()
        if command == 'q':
            x += 30
        elif command == 'w':
            x -= 30
        elif command == 'o':
            y += 10
        elif command == 'p':
            y -= 10
        elif command == 'b':
            break

        robot_command_client(x, y)

    sys.exit(1)
