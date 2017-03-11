#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

'''
Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest
object for you (you're free to pass in your own instead). The return value is an AddTwoIntsResponse object.
If the call fails, a rospy.ServiceException may be thrown, so you should setup the appropriate try/except block.
'''
