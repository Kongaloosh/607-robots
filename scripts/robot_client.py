#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import robot_command

def robot_command_client(x, y):
    rospy.wait_for_service('robot_command')
    try:
        command_service = rospy.ServiceProxy('robot_command', robot_command)
        resp1 = command_service(x, y)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    robot_command_client(x, y)
