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
from pysrc.algorithms.tdcontrol.onpolicy.sarsa import SARSA
from pysrc.algorithms.tdcontrol.onpolicy.actor_critic import ActorCritic, ContinuousActorCritic

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
        self.controller_publisher = rospy.Publisher('control_publisher' + name, td_control_msg, queue_size=10)
        self.kanerva = BaseKanervaCoder(
            _startingPrototypes=self.memory_size,
            _dimensions=1,
            _numActiveFeatures=self.active_features)
        self.control = td_control
        # rospy.init_node('robot_command_talker', anonymous=True)                             # initializes node with name

    def step(self, data):
        data = self.construct_obs(data)
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phi_next = self.kanerva.get_features(data)
        action_next = self.control.get_action(phi_next)
        if self.phi is not None:
            self.control.step(self.phi, reward, phi_next, self.gamma, self.lmbda, gnext)
        else:
            self.control.action = self.control.get_action(phi_next)

        self.phi = phi_next
        self.action = action_next
        self.gamma = gnext
        # need a bottleneck to throttle things
        self.controller_publisher.publish(
            reward,
            action_next
        )
        self.command(action_next)


    @staticmethod
    def command(action):
        print(action)
        rospy.wait_for_service('robot_controller')
        if action == 1:
            x = 1
            y = 1
        else:
            x = -1
            y = 1
        try:
            command_service = rospy.ServiceProxy('robot_controller', robot_command)
            resp1 = command_service(x, y, action)
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


class TDRobot_continuous(object):
    def __init__(self, elegibility_lambda, td_control, reward_factory, gamma, gamma_factory, name=""):
        self.memory_size = 2 ** 10
        self.active_features = 10
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.lmbda = elegibility_lambda
        self.gamma = gamma
        self.phi = None
        self.last_estimate = 0
        self.action = None
        self.mean_publisher = rospy.Publisher('mean_publisher' + name, td_control_msg, queue_size=10)
        self.sigma_publisher = rospy.Publisher('sigma_publisher' + name, td_control_msg, queue_size=10)
        self.kanerva = BaseKanervaCoder(
            _startingPrototypes=self.memory_size,
            _dimensions=1,
            _numActiveFeatures=self.active_features)
        self.control = td_control
        # rospy.init_node('robot_command_talker', anonymous=True)                             # initializes node with name

    def step(self, data):
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phi_next = self.kanerva.get_features(self.construct_obs(data))
        action_next = self.control.get_action(phi_next)
        if self.phi and self.action:
            self.control.step(self.phi, reward, phi_next, self.gamma, self.lmbda, gnext)
        self.phi = phi_next
        self.action = action_next
        self.gamma = gnext
        # need a bottleneck to throttle things
        self.sigma_publisher_publisher.publish(
            reward,
            action_next
        )
        self.command(action_next)

    @staticmethod
    def command(action):
        rospy.wait_for_service('robot_controller')
        x = action
        y = 1
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
    #    continuous_actor_critic = ContinuousActorCritic(2**10, 0.005,0.005, 0.0005, 1)
    #    robot = TDRobot_continuous(0.4,continuous_actor_critic,load_2,1,constant,"_continuous_actor_critic")
    actor_critic = ActorCritic(2 ** 10, 2, 0.005, 0.005, 0.0005, 1)
    robot = TDRobot(0.3, 0.4, actor_critic, load_2, 0.9, constant, name="_sarsa")
    # sarsa = SARSA(2**10, 2, 0.3, 10)
    # robot = TDRobot(0.3, 0.4, sarsa, load_2, 0.9, constant, name="_sarsa")

    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state, robot.step)  # subscribes to chatter and calls the callback
    rospy.spin()
