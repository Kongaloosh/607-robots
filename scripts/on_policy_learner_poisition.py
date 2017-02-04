#!/usr/bin/env python

from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.verifier import OnlineVerifier
import rospy
import numpy as np
from beginner_tutorials.msg import servo_state, verifier, gvf

__author__ = 'kongaloosh'


# todo: publish values
# todo: integrate online verifier
# todo: give the messages the right info


class OnPolicyPredictor(object):
    def __init__(self):
        self.num_tilings = 10
        self.memory_size = 2 * 10
        self.lmbda = 0.999
        self.gamma = 0.9
        self.phi = None
        self.tdr = TDR(
            number_of_features=self.memory_size,
            step_size=0.01,
            active_features=self.num_tilings
        )
        self.verifier = OnlineVerifier(rlGamma=self.gamma)

        self.verifier_publisher = rospy.Publisher('position_verifier', verifier, queue_size=10)
        rospy.init_node('position_verifier', anonymous=True)

        self.gvf_publisher = rospy.Publisher('position_predictor', gvf, queue_size=10)
        rospy.init_node('position_predictor', anonymous=True)

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
        ]                                       # form a state from new observations
                                                # todo: multiply by the number of bins
        f = np.array(
            getTiles(
                numtilings=self.num_tilings,    # the number of tilings in your tilecoder
                memctable=self.memory_size,     # the amount of memory for each tilecoder
                floats=state                    # the observations from the robot
            )
        )
        f = np.concatenate(([1], f))            # add a bias

        phi_next = np.zeros(self.memory_size)   # make a new feature vector
        for i in f:                             # update phi so that...
            phi_next[i] = 1                     # all active features are 1

        if self.phi is not None:
            reward = data.load_2
            self.tdr.step(
                self.phi,
                reward,
                phi_next,
                self.gamma,
                self.lmbda,
                self.gamma
            )

            prediction = self.tdr.estimate(self.phi)

            self.verifier.update_reward(reward)
            self.verifier.update_prediction(prediction)     # update the prediction
            self.verifier_publisher.Publish(                # publish the verifier's info (offset by horizon)
                self.verifier.synced_prediction(),
                self.verifier.calculate_currente_return(),
                self.verifier.calculate_current_error()
            )

            self.gvf_publisher.Publish(             # publish the most recent predictions
                prediction,
                prediction/(1./(1.-self.gamma))     # prediction normalized by the timescale of the horizon
            )
        self.phi = phi_next                         # update phi


def listener(predictor):
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state,
                     predictor.handle_obs)                 # subscribes to chatter and calls the callback
    rospy.spin()                                           # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    on_policy_predictor = OnPolicyPredictor()
    listener(on_policy_predictor)
