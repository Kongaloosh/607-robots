#!/usr/bin/env python

from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.kanerva_coding import BaseKanervaCoder
from pysrc.algorithms.tdprediction.discount_rates import *
from pysrc.algorithms.tdprediction.reward_functions import *
from pysrc.utilities.verifier import OnlineVerifier, UDE, RUPEE
import rospy
import numpy as np
from beginner_tutorials.msg import servo_state, verifier, gvf

__author__ = 'kongaloosh'


class Horde(object):
    def __init__(self):
        self.predictors = []

    def add_learner(self, step_size, elegibility_lambda, gamma, learner):
        # each should have a v
        self.predictors.append(learner(step_size, elegibility_lambda, gamma, learner))

    def update(self, data):
        """"""
        data = self.construct_obs(data)
        [learner.update(data) for learner in self.predictors]

    def construct_obs(self, data):
        return data


class RobotHorde(Horde):
    def construct_obs(self, data):
        # also want to construc long-term stuff
        data = [
            data.load_2,
            data.temperature_2,
            data.voltage_2,
            data.moving_2,
            data.position_2,
            data.angle_2,
            data.vel_command_2,
            data.load_3,
            data.temperature_3,
            data.voltage_3,
            data.moving_3,
            data.position_3,
            data.angle_3,
            data.vel_command_3,
            data.command,
        ]
        for learner in self.predictors:
            data.append(learner.last_estimate())


class GVF(object):
    def __init__(self, step_size, elegibility_lambda, gamma, learner):
        self.memory_size = 2 ** 10
        self.lmbda = elegibility_lambda
        self.gamma = gamma
        self.phi = None
        self.verfier = OnlineVerifier(rlGamma=self.gamma)
        self.learner = learner()
        self.kanerva = BaseKanervaCoder()


class OnPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward, gamma):
        super.__init__(step_size, elegibility_lambda, learner)
        self.reward_factory = reward
        self.gamma_factory = gamma
        self.gvf_publisher = rospy.Publisher('position_predictor', gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('position_verifier', verifier, queue_size=10)
        self.verfier = OnlineVerifier(self.gamma)

    def update(self, data):
        # get the new gamma
        gnext = self.gamma_factory(data)
        reward = self.reward_factory(data)
        phinext = self.kanerva(data)
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext)
            prediction = self.learner.estimate(phinext)
            self.verfier.update_all(gamma=gnext, reward=reward, prediction=prediction)
            try:
                self.gvf_publisher(
                    prediction,
                    prediction / (1. / (1. - gnext))
                )
            except:
                pass

            try:
                self.gvf_verifier_publisher.publish(
                    self.verfier.synced_prediction(),
                    self.verfier.calculate_currente_return(),
                    abs(self.verfier.calculate_current_error())
                )
            except:
                pass

        self.gamma = gnext
        self.phi = phinext


class OffPolicyGVF(GVF):

    def __init__(self, step_size, elegibility_lambda, learner, reward, gamma):
        super.__init__(step_size, elegibility_lambda, learner)
        self.reward_factory = reward
        self.gamma_factory = gamma
        self.gvf_publisher = rospy.Publisher('position_predictor', gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('position_verifier', verifier, queue_size=10)
        self.verfier = RUPEE(self.gamma)

    def update(self, data):
        # get the new gamma
        gnext = self.gamma_factory(data)
        reward = self.reward_factory(data)
        phinext = self.kanerva(data)
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext)
            prediction = self.learner.estimate(phinext)
            delta = reward + gnext * self.learner.estimate(phinext) - self.learner.estimate(self.phi)
            rupee = self.verfier.update_all(self.learner.z, delta, phinext)
            try:
                self.gvf_publisher(
                    prediction,
                    prediction / (1. / (1. - gnext)),
                )
            except:
                pass

            try:
                self.gvf_verifier_publisher.publish(
                    prediction,
                    rupee
                )
            except:
                pass

        self.gamma = gnext
        self.phi = phinext
