#!/usr/bin/env python

from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.algorithms.tdprediction.onpolicy.tdbd import TDBD
from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.verifier import OnlineVerifier, UDE, RUPEE
import rospy
import numpy as np
from beginner_tutorials.msg import servo_state, verifier, gvf

__author__ = 'kongaloosh'


class OnPolicyPredictor(object):
    def __init__(self):
        self.num_tilings = 10
        self.memory_size = 2 ** 10
        self.lmbda = 0.99
        self.gamma = 0.95
        self.phi = None
        self.tdr = TDR(
            number_of_features=self.memory_size,
            step_size=0.01,
            active_features=self.num_tilings
        )
        self.verifier = OnlineVerifier(rlGamma=self.gamma)
        self.ude = UDE(0.01)
        self.rupee = RUPEE(2**10, 0.01 * 5, 0.001)

        self.verifier_publisher = rospy.Publisher('load_verifier', verifier, queue_size=10)
        self.gvf_publisher = rospy.Publisher('load_predictor', gvf, queue_size=10)

        self.position_trace = 0
        self.last_load = 0

    def handle_obs(self, data):
        """ takes the observations from the words """

        self.position_trace = data.load_2 - self.last_load + self.position_trace * 0.8

        if data.command > 0:
            gnext = 0
        else:
            gnext = 0.9

        state = np.array([
            # data.voltage_2 / 16.,
            data.load_2 / 1024.,
            data.position_2 / 1024.,
            self.position_trace + 1024 / 2048,
            # (data.vel_command_2 + 2)/4.,
            data.command
        ])  # form a state from new observations
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
            reward = data.load_2
            self.tdr.step(
                self.phi,
                reward,
                phi_next,
                self.gamma,
                self.lmbda,
                gnext
            )

            prediction = self.tdr.estimate(self.phi)
            try:
                self.verifier.update_reward(reward)
                self.verifier.update_prediction(prediction)  # update the prediction
                self.verifier.update_gamma(self.gamma)
                delta = delta = reward + gnext * self.tdr.estimate(phi_next) - self.tdr.estimate(self.phi)
                ude_error = self.ude.update(delta)
                rupee_error = self.rupee(delta, self.tdr.z, self.phi)
                self.verifier_publisher.publish(  # publish the verifier's info (offset by horizon)
                      self.verifier.synced_prediction(),
                      self.verifier.calculate_currente_return(),
                      abs(self.verifier.calculate_current_error()),
                      ude_error,
                      rupee_error
                                                  )
            except IndexError:
                pass
            self.gvf_publisher.publish(
                 prediction,
                 prediction / (1. / (1. - self.gamma)))
        self.phi = phi_next  # update phi
        self.gamma = gnext
        self.last_load = data.load_2


def listener(predictor):
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state,
                     predictor.handle_obs)  # subscribes to chatter and calls the callback
    rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    on_policy_predictor = OnPolicyPredictor()
    listener(on_policy_predictor)
