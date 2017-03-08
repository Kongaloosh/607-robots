#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import robot_command
from beginner_tutorials.msg import robot_command_state
import random
import time
from pysrc.algorithms.tdprediction.discount_rates import *
from pysrc.algorithms.tdprediction.reward_functions import *
from beginner_tutorials.msg import servo_state, verifier, gvf, state, td_control_msg
from pysrc.utilities.kanerva_coding import BaseKanervaCoder

__author__ = 'kongaloosh'


class TDRobot(object):

    def __init__(self, step_size, elegibility_lambda, td_control, reward_factory, gamma, gamma_factory, name=""):
        self.memory_size = 2 ** 10
        self.active_features = 10
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.lmbda = elegibility_lambda
        self.step_size = step_size
        self.gamma = gamma
        self.phi = None
        self.last_estimate = 0
        self.action = None
        self.kanerva = BaseKanervaCoder(
            _startingPrototypes=self.memory_size,
            _dimensions=1,
            _numActiveFeatures=self.active_features)
        self.control = td_control
        pub = rospy.Publisher('robot_obeservation', td_control_msg, queue_size=10)     # publishing to
        rospy.init_node('robot_command_talker', anonymous=True)                             # initializes node with name

    def step(self, data):
        gnext = self.gamma_factory(data, self.gamma)
        reward = self.reward_factory(data)
        phi_next = self.kanerva.get_features(self.construct_obs(data))
        action_next = self.control.get_action(phi_next)
        if self.phi and self.action:
           self.control.step(self.phi, reward, phi_next, self.gamma, self.lmbda, gnext)
        self.phi = phi_next
        self.robot_command_client(self.action)
        self.action = action_next
        self.gamma = gnext
        # need a bottleneck to throttle things
        self.command(action_next)

    @staticmethod
    def command(action):
        rospy.wait_for_service('robot_controller')
        if action == 1:
            x = 1
            y = 1
        else:
            x = -1
            y = 1
        try:
            command_service = rospy.ServiceProxy('robot_controller', robot_command)
            resp1 = command_service(x, y, command)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def robot_command_client(command):
        #todo: command factories
        rospy.wait_for_service('robot_controller')
        try:
            command_service = rospy.ServiceProxy('robot_controller', robot_command)
            resp1 = command_service(x, y, command)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def construct_obs(data):
        data = [
            data.load_2,
            data.temperature_2,
            data.voltage_2,
            data.is_moving_2,
            data.position_2,
            data.angle_2,
            data.vel_command_2,
            data.load_3,
            data.temperature_3,
            data.voltage_3,
            data.is_moving_3,
            data.position_3,
            data.angle_3,
            data.vel_command_3,
            data.command,
        ]
        return data


class TDRobot_Continuous(object):

    def __init__(self, step_size, elegibility_lambda, td_control, reward_factory, gamma, gamma_factory, name=""):
        self.memory_size = 2 ** 10
        self.active_features = 10
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.lmbda = elegibility_lambda
        self.step_size = step_size
        self.gamma = gamma
        self.phi = None
        self.last_estimate = 0
        self.action = None
        self.kanerva = BaseKanervaCoder(
            _startingPrototypes=self.memory_size,
            _dimensions=1,
            _numActiveFeatures=self.active_features)
        self.control = td_control
        pub = rospy.Publisher('robot_obeservation', td_control_msg, queue_size=10)     # publishing to
        rospy.init_node('robot_command_talker', anonymous=True)                             # initializes node with name

    def command(self):
        rospy.wait_for_service('robot_controller')
        try:
            command_service = rospy.ServiceProxy('robot_controller', robot_command)
            resp1 = command_service(x, y, command)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def step(self, data):
        gnext = self.gamma_factory(data, self.gamma)
        reward = self.reward_factory(data)
        phi_next = self.kanerva.get_features(self.construct_obs(data))
        action_next = self.control.get_action(phi_next)
        if self.phi and self.action:
            action = self.control.step(self.phi, reward, phi_next, self.gamma, self.lmbda, gnext)

        self.phi = phi_next
        self.robot_command_client(self.action)
        self.action = action_next
        self.gamma = gnext
        # reward
        # command
        # need a bottleneck to throttle things


    @staticmethod
    def robot_command_client(command):
        #todo: command factories
        rospy.wait_for_service('robot_controller')
        try:
            command_service = rospy.ServiceProxy('robot_controller', robot_command)
            resp1 = command_service(x, y, command)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def construct_obs(data):
        data = [
            data.load_2,
            data.temperature_2,
            data.voltage_2,
            data.is_moving_2,
            data.position_2,
            data.angle_2,
            data.vel_command_2,
            data.load_3,
            data.temperature_3,
            data.voltage_3,
            data.is_moving_3,
            data.position_3,
            data.angle_3,
            data.vel_command_3,
            data.command,
        ]
        return data

if __name__ == "__main__":
    robot = TDRobot()
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state, robot.step)  # subscribes to chatter and calls the callback

