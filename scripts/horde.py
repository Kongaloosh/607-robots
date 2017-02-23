#!/usr/bin/env python

from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.kanerva_coding import BaseKanervaCoder
from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.algorithms.tdprediction.offpolicy.gtd import GTD, GTDR
from pysrc.algorithms.tdprediction.offpolicy.policy import *
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

    def add_learner(self, learner):
        # each should have a v
        self.predictors.append(learner)

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
        for learner in self.predictors:
            data.append(learner.last_estimate)
	return data

class GVF(object):
    def __init__(self, step_size, elegibility_lambda, gamma, learner):
        self.memory_size = 2 ** 10
        self.active_features = 10
	self.lmbda = elegibility_lambda
        self.step_size = step_size
        self.gamma = gamma
        self.phi = None
        self.verifier = OnlineVerifier(rlGamma=self.gamma)
        self.learner = learner
        self.last_estimate = 0
	self.kanerva = BaseKanervaCoder(
            _startingPrototypes=self.memory_size,
            _dimensions=1,
            _numActiveFeatures=self.active_features)


class OnPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory):
        super(OnPolicyGVF, self).__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.gvf_publisher = rospy.Publisher('position_predictor', gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('position_verifier', verifier, queue_size=10)
        self.verfier = OnlineVerifier(self.gamma)

    def update(self, data):
        # get the new gamma
	gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phinext = self.kanerva.get_features(data)
        print phinext
	if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext)
            self.last_setimate = self.learner.estimate(phinext)
            self.verfier.update_all(gamma=gnext, reward=reward, prediction=self.last_estimate)
            try:
                self.gvf_publisher(
                    self.last_estimate,
                    self.last_estimate / (1. / (1. - gnext))
                )
            except IndexError:
                pass

            try:
                self.gvf_verifier_publisher.publish(
                    self.verfier.synced_prediction(),
                    self.verfier.calculate_currente_return(),
                    abs(self.verfier.calculate_current_error())
                )
            except IndexError:
                pass

        self.gamma = gnext
        self.phi = phinext


class OffPolicyGVF(GVF):

    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory):
        super.__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
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


def listener():
    horde = RobotHorde()
    horde.add_learner(learner=OnPolicyGVF(0.3, 0.9, TDR(2**10, 0.3, 10), poisiton_2, 0.98, end_when_stationary_2))
    # horde.add_learner(learner=OffPolicyGVF(0.3, 0.9, TDR, poisiton_2, end_when_stationary_3))
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state,
                     horde.update)  # subscribes to chatter and calls the callback
    rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
   listener()
