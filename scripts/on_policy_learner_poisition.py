#!/usr/bin/env python

from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.utilities.tiles import loadTiles, getTiles
import rospy
import numpy as np
from beginner_tutorials.msg import servo_state

__author__ = 'kongaloosh'

# todo: publish values
# todo: integrate online verifier
# todo: give the messages the right info

class OnPolicyPredictor(object):
    def __init__(self):
        self.num_tilings = 10
        self.memory_size = 2*10
        self.lmbda = 0.999
        self.gamma = 0.9
        self.phi = None
        self.tdr = TDR(
            number_of_features=self.memory_size,
            step_size=0.001,
            active_features=self.num_tilings
        )

    def handle_obs(self, data):
        """ takes the observations from the words """
        # todo: norm the values
        state = [
            data.voltage_2,
            data.voltage_3,
            data.load_2,
            data.load_3,
            data.is_moving_2,
            data.is_moving_3
        ]                                                   # form a state from new observations
        # todo: multiply by the number of bins
        f = np.array(
                        getTiles(
                            numtilings=self.num_tilings,    # the number of tilings in your tilecoder
                            memctable=self.memory_size,     # the amount of memory for each tilecoder
                            floats=state                    # the observations from the robot
                        )
        )
        f = np.concatenate(([1], f))

        phi_next = np.zeros(self.memory_size)               # make a new feature vector
        for i in f:                                         # update phi so that...
            phi_next[i] = 1                                 # all active features are 1

        if self.phi is not None:
            self.tdr.step(self.phi, data.load_2, phi_next, self.gamma, self.lmbda, self.gamma)
	    print(data.load_2, np.dot(self.tdr.estimate(), self.phi))
        self.phi = phi_next                                 # update phi


def listener(predictor):
    rospy.init_node('on_policy_listener', anonymous=True)     # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state, predictor.handle_obs)   # subscribes to chatter and calls the callback with the data as arg
    rospy.spin()                                    # keeps python from exiting until this node is stopped

if __name__ == '__main__':
    on_policy_predictor = OnPolicyPredictor()
    listener(on_policy_predictor)
