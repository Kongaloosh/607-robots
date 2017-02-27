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
        self.ude = UDE(self.step_size * 10)
        self.learner = learner
        self.last_estimate = 0
        self.kanerva = BaseKanervaCoder(
            _startingPrototypes=self.memory_size,
            _dimensions=1,
            _numActiveFeatures=self.active_features)


class OnPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory, name=""):
        super(OnPolicyGVF, self).__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.gvf_publisher = rospy.Publisher('on_policy_predictor' + name, gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('on_policy_verifier' + name, verifier, queue_size=10)
        self.verfier = OnlineVerifier(self.gamma)
	self.rupee = RUPEE(self.memory_size, self.step_size * 5, 0.001)

    def update(self, data):
        # get the new gamma
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phinext = self.kanerva.get_features(data)
        # print np.where(phinext > 0)
        # print reward
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext)
            self.last_estimate = self.learner.last_estimate()
            # print np.dot(self.learner.th, phinext)
            self.verfier.update_all(gamma=gnext, reward=reward, prediction=self.last_estimate)
            delta = reward + gnext * self.learner.estimate(phinext) - self.learner.estimate(self.phi)
            ude_error = self.ude.update(delta)
            rupee_error = self.rupee.update(self.learner.z, delta, phinext)
	    # todo: you could factor all of this out
            self.gvf_publisher.publish(
                self.last_estimate,
                self.last_estimate / (1. / (1. - gnext))
            )

            try:
                self.gvf_verifier_publisher.publish(
                    self.verfier.synced_prediction(),
                    self.verfier.calculate_currente_return(),
                    abs(self.verfier.calculate_current_error()),
                    ude_error,
		    rupee_error
                )
            except IndexError:
                pass

        self.gamma = gnext
        self.phi = phinext


class OffPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory, name=""):
        super(OffPolicyGVF, self).__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.gvf_publisher = rospy.Publisher('off_policy_predictor' + name, gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('off_policy_verifier' + name, verifier, queue_size=10)
        self.verfier = RUPEE(self.memory_size, self.step_size * 5, 0.001)

    def update(self, data):
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phinext = self.kanerva.get_features(data)
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext, data[14])
            prediction = self.learner.estimate(phinext)
            delta = reward + gnext * self.learner.estimate(phinext) - self.learner.estimate(self.phi)
            rupee = self.verfier.update(self.learner.z, delta, phinext)
            ude_error = self.ude.update(delta)
            self.gvf_publisher.publish(
                prediction,
                prediction / (1. / (1. - gnext)),
            )
            try:
                self.gvf_verifier_publisher.publish(
                    prediction,
                    0,
                    0,
                    ude_error,
                    rupee
                )
            except IndexError:
                pass

        self.gamma = gnext
        self.phi = phinext


def listener():
    horde = RobotHorde()
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), angle_2, 0.98, constant, name="_0"))
    horde.add_learner(
        learner=OffPolicyGVF(0.03, 0.9, GTDR(2 ** 10, 0.03, moving_left_1, 10), angle_2, 0.99, constant, name="_1"))
    horde.add_learner(
        learner=OffPolicyGVF(0.03, 0.9, GTDR(2 ** 10, 0.03, moving_right_1, 10), angle_2, 0.5, constant, name="_2"))
    horde.add_learner(
        learner=OffPolicyGVF(0.03, 0.9, GTDR(2 ** 10, 0.03, moving_left_2, 10), angle_2, 0.9, constant, name="_3"))
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), is_moving_2, 0.98, constant, name="_4"))
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), poisiton_2, 0.98, constant, name="_5"))
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), voltage_2, 0.98, constant, name="_6"))
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), temperature_2, 0.98, constant, name="_7"))
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), command, 0.98, constant, name="_8"))
    horde.add_learner(learner=OnPolicyGVF(0.03, 0.9, TDR(2 ** 10, 0.03, 10), load_2, 0.98, constant, name="_9"))

    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state,
                     horde.update)  # subscribes to chatter and calls the callback
    rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    listener()
