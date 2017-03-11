#!/usr/bin/env python

__author__ = 'kongaloosh'
from updated_lib_robotis_hack import *
from beginner_tutorials.msg import *
from beginner_tutorials.srv import *
import numpy as np
import sys
import rospy
import math
import time


class Robot(object):
    def __init__(self):

        serial_port = '/dev/ttyUSB0'
        print(serial_port)
        self.D = USB2Dynamixel_Device(dev_name=serial_port, baudrate=1000000)
        self.s_list = find_servos(self.D)
        self.s1 = Robotis_Servo(self.D, self.s_list[0])
        self.s2 = Robotis_Servo(self.D, self.s_list[1])
        self.last_vel_command_2 = 0
        self.last_vel_command_3 = 0
        self.last_command = 0
        print("servos found")
        self.s1.kill_cont_turn()
        self.s2.kill_cont_turn()
        self.s1.move_to_encoder(512)
        self.s2.move_to_encoder(512)
        rospy.init_node('robot', anonymous=True)
        print("robot node made")
        # Publishes robot state
        self.observation_publisher = rospy.Publisher('robot_observations', servo_state, queue_size=10)
        print("obs publisher")
        # service for controlling servos
        self.robot_controller_server = rospy.Service('robot_controller', robot_command, self.command_handler)
        rospy.wait_for_service('robot_controller')
        print("robot controller")
        self.start_controller = rospy.ServiceProxy('robot_controller', robot_command, self.command_handler)
        print("controller service started")
        # timer which defines callback for the publisher
        rospy.Timer(rospy.Duration(1.0 / 10.), self.observation_callback)

        time.sleep(.2)
        self.s1.init_cont_turn()
        self.s2.init_cont_turn()

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
                    self.s1.read_angle(),
                    self.last_vel_command_2,
                    self.s2.read_load(),
                    self.s2.read_temperature(),
                    self.s2.read_voltage(),
                    self.s2.is_moving(),
                    self.s2.read_encoder(),
                    self.s2.read_angle(),
                    self.last_vel_command_3,
                    self.last_command
                )
                return state
            except:
                pass

    def command_handler(self, request):
        while True:
            self.last_vel_command_2 = request.goal_pos_2
            self.last_vel_command_3 = request.goal_pos_3
            self.s1.set_angvel(request.goal_pos_2)
            self.s2.set_angvel(request.goal_pos_3)
            self.last_command = request.command
            return request.goal_pos_2, request.goal_pos_3


if __name__ == '__main__':
    try:
        robot = Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
