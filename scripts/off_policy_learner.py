#!/usr/bin/env python

from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.algorithms.tdprediction.offpolicy.gtd import GTD, GTDR
from pysrc.algorithms.tdprediction.offpolicy.policy import Policy
from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.verifier import OnlineVerifier
import rospy
import numpy as np
from beginner_tutorials.msg import servo_state, verifier, gvf

__author__ = 'kongaloosh'


class OnPolicyPredictor(object):
    def __init__(self):
        self.num_tilings = 10
        self.memory_size = 2 ** 10
        self.lmbda = 0.9
        self.gamma = 0.95
        self.phi = None
        self.behavior_policy = Policy(self.memory_size, 2)
        self.tdr = GTDR(
            number_of_features=self.memory_size,
            step_size=0.01,
            target_policy=None,
            number_of_active_features=self.num_tilings
        )
        self.gvf_publisher = rospy.Publisher('position_predictor', gvf, queue_size=10)

        self.position_trace = 0
        self.las_pos = 0

    def handle_obs(self, data):
        """ takes the observations from the words """

        self.position_trace = data.position_2 - self.las_pos + self.position_trace * 0.8

        state = np.array([
            # data.voltage_2 / 16.,
            # data.load_2 / 1024.,
            data.position_2 / 1024.,
             (self.position_trace + 1024) /2048. ,
            data.command
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
            reward = data.load_2
            self.behavior_policy.action_state_update(data.command, self.phi)
            self.tdr.step(
                self.phi,
                reward,
                phi_next,
                self.gamma,
                self.lmbda,
                self.gamma,
                self.behavior_policy,
                data.command
            )
            prediction = self.tdr.estimate(self.phi)

            try:
                self.gvf_publisher.publish(  # publish the most recent predictions
                                             prediction,
                                             prediction / (1. / (1. - self.gamma))
                                             # prediction normalized by the timescale of the horizon
                                             )
            except:
                pass
        self.phi = phi_next  # update phi
        self.las_pos = data.position_2


def listener(predictor):
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state,
                     predictor.handle_obs)  # subscribes to chatter and calls the callback
    rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    on_policy_predictor = OnPolicyPredictor()
    listener(on_policy_predictor)
