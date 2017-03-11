#!/usr/bin/env python

from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.algorithms.tdprediction.offpolicy.gtd import GTD, GTDR
from pysrc.algorithms.tdprediction.offpolicy.policy import *
from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.verifier import OnlineVerifier
import rospy
import numpy as np
from beginner_tutorials.msg import servo_state, verifier, gvf
from pysrc.utilities.verifier import UDE, RUPEE

__author__ = 'kongaloosh'


class OffPolicyPredictor(object):
    def __init__(self):
        self.num_tilings = 10
        self.memory_size = 2 ** 10
        self.lmbda = 0.99
        self.gamma = 0.95
        self.phi = None
        self.tdr = GTDR(
            number_of_features=self.memory_size,
            step_size=0.01,
            target_policy=moving_right_1,
            number_of_active_features=self.num_tilings
        )
        self.gvf_publisher = rospy.Publisher('position_predictor', gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('off_policy_verifier', verifier, queue_size=10)
        self.ude = UDE(0.01)
        self.rupee = RUPEE(2**10, 0.01 * 5, 0.001)
        self.position_trace = 0
        self.las_pos = 0
        self.vel_trace = 0

    def handle_obs(self, data):
        """ takes the observations from the words """

        self.position_trace = data.position_2 - self.las_pos + self.position_trace * 0.8
        self.vel_trace = data.position_2 - self.las_pos + self.vel_trace * 0.8

        state = np.array([
            # data.voltage_2 / 16.,
            # data.load_2 / 1024.,
            data.position_2 / 1024.,
            (self.position_trace + 1024.) / 2048.,
            (self.vel_trace + 1024.) / 2048.,
            # data.command
        ])  # form a state from new observations
        print(state)
        state *= 10  # multiply by the number of bins
        f = np.array(
            getTiles(
                numtilings=self.num_tilings,  # the number of tilings in your tilecoder
                memctable=self.memory_size,  # the amount of memory for each tilecoder
                floats=state  # the observations from the robot
            )
        )
        f = np.concatenate(([1], f))  # add a bias

        phi_next = np.zeros(self.memory_size)  # make a new feature vector
        for i in f:  # update phi so that...
            phi_next[i] = 1  # all active features are 1

        if self.phi is not None:
            reward = data.position_2
            self.tdr.step(
                self.phi,
                reward,
                phi_next,
                self.gamma,
                self.lmbda,
                self.gamma,
                data.command
            )
            prediction = self.tdr.estimate(self.phi)
            delta = reward + self.gamma * self.tdr.estimate(phi_next) - self.tdr.estimate(self.phi)
            ude_error = self.ude.update(delta)
            rupee_error = self.rupee.update(delta, self.tdr.z, self.phi)
            self.gvf_publisher.publish(  # publish the most recent predictions
                                         prediction,
                                         prediction / (1. / (1. - self.gamma))
                                         # prediction normalized by the timescale of the horizon
                                         )

            try:
                self.gvf_verifier_publisher.publish(
                    prediction,
                    0,
                    0,
                    ude_error,
                    rupee_error
                )
            except IndexError:
                pass
        self.phi = phi_next  # update phi
        self.las_pos = data.position_2


def listener(predictor):
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state,
                     predictor.handle_obs)  # subscribes to chatter and calls the callback
    rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    on_policy_predictor = OffPolicyPredictor()
    listener(on_policy_predictor)
