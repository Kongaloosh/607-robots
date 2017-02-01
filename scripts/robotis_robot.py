#!/usr/bin/env python

__author__ = 'kongaloosh'
from updated_lib_robotis_hack import *
from beginner_tutorials.msg import *
from beginner_tutorials.srv import *
import dynamixel
import sys
import rospy


class Robot(object):

    def __init__(self):

        serial_port = '/dev/ttyUSB0'
	self.D = USB2Dynamixel_Device(dev_name=serial_port, baudrate=1000000)
        self.s_list = find_servos(self.D)
	self.s1 = Robotis_Servo(self.D, self.s_list[0])
	self.s2 = Robotis_Servo(self.D, self.s_list[1])
        rospy.init_node('robot', anonymous=True)
        # Publishes robot state
        self.observation_publisher = rospy.Publisher('robot_observations', servo_state, queue_size=10)
        # service for controlling servos
        self.robot_controller_server = rospy.Service('robot_controller', robot_command, self.command_handler)
        rospy.wait_for_service('robot_controller')
        self.start_controller = rospy.ServiceProxy('robot_controller', robot_command, self.command_handler)
        self.start_controller(512,512)
	# timer which defines callback for the publisher
        rospy.Timer(rospy.Duration(1.0/10), self.observation_callback)

    def observation_callback(self, timer):
        # Make a header ???
        self.observation_publisher.publish(*self.get_observations())

    def get_observations(self):
        """Turns the Current Values of the Robot Servos Into a Tuple to be Passed as a Message"""
	while True:        
	    try:	
		state = (
			self.s1.read_load(),
			self.s1.read_temperature(),
			self.s1.read_voltage(),
			self.s1.is_moving(),
			self.s1.read_encoder(),
			self.s2.read_load(),
			self.s2.read_temperature(),
			self.s2.read_voltage(),
			self.s2.is_moving(),
			self.s2.read_encoder(),
		)
		return state
	    except: pass

    def command_handler(self, request):
	while True:
	    try:	
		self.s1.move_to_encoder(request.goal_pos_2)
		self.s2.move_to_encoder(request.goal_pos_3)
		return request.goal_pos_2, request.goal_pos_3
	    except: pass

if __name__ == '__main__':
    try:
        robot = Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
