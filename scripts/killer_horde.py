#!/usr/bin/env python

import rospy
import numpy as np
from beginner_tutorials.msg import servo_state, verifier, gvf, state
from horde import Horde, GVF
from pysrc.utilities.tiles import loadTiles, getTiles
from pysrc.utilities.kanerva_coding import BaseKanervaCoder
from pysrc.algorithms.tdprediction.onpolicy.tdr import TDR
from pysrc.algorithms.tdprediction.onpolicy.tdbd import TDBDR
from pysrc.algorithms.tdprediction.offpolicy.gtd import GTD, GTDR
from pysrc.algorithms.tdprediction.offpolicy.policy import *
from pysrc.algorithms.tdprediction.discount_rates import *
from pysrc.algorithms.tdprediction.reward_functions import *
from pysrc.utilities.verifier import OnlineVerifier, UDE, RUPEE
from beginner_tutorials.msg import servo_state, verifier, gvf, state, daemon_killer

__author__ = 'kongaloosh'

reward_functions = [load_2, temperature_2, voltage_2, is_moving_2, poisiton_2, angle_2, vel_command_2, command]


class OnPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory, name=""):
        super(OnPolicyGVF, self).__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.gvf_publisher = rospy.Publisher('on_policy_predictor' + name, gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('on_policy_verifier' + name, verifier, queue_size=10)
        self.verfier = OnlineVerifier(self.gamma)
        self.rupee = RUPEE(self.memory_size, self.step_size * 5, 0.001)
        self.rupee_last = 0
        self.age = 0
        self.dead = False

    def update(self, obs, data):
        # get the new gamma
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phinext = obs
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext)
            self.last_estimate = self.learner.last_estimate()
            self.verfier.update_all(gamma=gnext, reward=reward, prediction=self.last_estimate)
            delta = reward + gnext * self.learner.estimate(phinext) - self.learner.estimate(self.phi)
            ude_error = self.ude.update(delta)
            self.rupee_last = self.rupee.update(self.learner.z, delta, phinext)
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
                    self.rupee_last
                )
            except IndexError:
                pass

            if self.dead:
                self.rupee_last = -1
        self.gamma = gnext
        self.phi = phinext
        self.age += 1


class OffPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory, name=""):
        super(OffPolicyGVF, self).__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.gvf_publisher = rospy.Publisher('off_policy_predictor' + name, gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('off_policy_verifier' + name, verifier, queue_size=10)
        self.rupee = RUPEE(self.memory_size, self.step_size * 5, 0.001)
        self.rupee_last = 0
        self.age = 0
        self.dead = False

    def update(self, obs, data):
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phinext = obs
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext, data.command)
            prediction = self.learner.estimate(phinext)
            delta = reward + gnext * self.learner.estimate(phinext) - self.learner.estimate(self.phi)
            self.rupee_last = self.rupee.update(self.learner.z, delta, phinext)
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
                    self.rupee_last
                )
            except IndexError:
                pass

            if self.dead:
                self.rupee_last = -1
        self.gamma = gnext
        self.phi = phinext
        self.age += 1


class DaemonKiller(Horde):
    def __init__(self):
        super(DaemonKiller, self).__init__()
        self.vel_trace = 0
        self.position_trace = 0
        self.min = 0
        self.max = 0
        self.last_pos = 0
        self.daemon_publisher = rospy.Publisher('daemon_killer_horde', daemon_killer, queue_size=10)
        self.age_threshold = 20000
        self.memory_size = 2**10
        self.num_tilings = 10

    def construct_obs(self, data):
        self.vel_trace = data.position_2 - self.last_pos + self.vel_trace * 0.8
        self.position_trace = data.position_2 + self.position_trace * 0.8
        self.last_pos = data.position_2
        data = np.array([
            data.position_2,
            self.position_trace,
            self.vel_trace,
        ])
        if self.min is not None and self.max is not None:
            self.min = np.minimum(self.min, data)
            self.max = np.maximum(self.max, data)
            data = (data + np.abs(self.min)) / (np.abs(self.min) + self.max)
            data = np.nan_to_num(data)
        else:
            self.min = data
            self.max = data
            data = np.ones(len(data))

        phi_next = np.zeros(self.memory_size)
        np.put(
            phi_next,
            np.concatenate(
                np.array(getTiles(
                    numtilings=self.num_tilings,  # the number of tilings in your tilecoder
                    memctable=self.memory_size - 1,  # the amount of memory for each tilecoder
                    floats=data * 10  # the observations from the robot
                )
                )
                ,
                int(2 ** 10)
            ),
            [1]
        )
        return phi_next

    def update(self, data):
        obs = self.construct_obs(data)
        [learner.update(obs, data) for learner in self.predictors]
        self.daemon_publisher.publish(
            [daemon.rupee_last for daemon in self.predictors],
            self.calc_rupee()
        )
        #self.kill()

    def kill(self):
        mean_rupees = self.fetch_rupee()
        np.put(mean_rupees, np.where(mean_rupees == 0), 1)
	kill = np.argmax(mean_rupees)
	if self.predictors[kill].age > self.age_threshold and mean_rupees[kill] > 0.75:
            print("Killed {0}".format(kill))
            # self.predictors.pop(kill)
            self.predictors[kill].dead = True

    def fetch_rupee(self):
        return np.array([daemon.rupee_last for daemon in self.predictors])

    def calc_rupee(self):
        a = np.array([daemon.rupee_last for daemon in self.predictors])
        return a / np.sum(a)


class TIDBDOnPolicyGVF(GVF):
    def __init__(self, step_size, elegibility_lambda, learner, reward_factory, gamma, gamma_factory, name=""):
        super(OnPolicyGVF, self).__init__(step_size, elegibility_lambda, gamma, learner)
        self.reward_factory = reward_factory
        self.gamma_factory = gamma_factory
        self.gvf_publisher = rospy.Publisher('on_policy_predictor' + name, gvf, queue_size=10)
        self.gvf_verifier_publisher = rospy.Publisher('on_policy_verifier' + name, verifier, queue_size=10)
        self.verfier = OnlineVerifier(self.gamma)
        self.rupee = RUPEE(self.memory_size, self.step_size * 5, 0.001)
        self.rupee_last = 0
        self.age = 0
        self.dead = False

    def update(self, obs, data):
        # get the new gamma
        gnext = self.gamma_factory(self.gamma, data)
        reward = self.reward_factory(data)
        phinext = obs
        if self.phi is not None:
            self.learner.step(self.phi, reward, phinext, self.gamma, self.lmbda, gnext)
            self.last_estimate = self.learner.last_estimate()
            self.verfier.update_all(gamma=gnext, reward=reward, prediction=self.last_estimate)
            delta = reward + gnext * self.learner.estimate(phinext) - self.learner.estimate(self.phi)
            ude_error = self.ude.update(delta)
            self.rupee_last = self.rupee.update(self.learner.z, delta, phinext)
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
                    self.rupee_last
                )
            except IndexError:
                pass

            if self.dead:
                self.rupee_last = -1
        self.gamma = gnext
        self.phi = phinext
        self.age += 1


class TIDBDDaemonKiller(Horde):

    def __init__(self):
        super(TIDBDDaemonKiller, self).__init__()
        self.vel_trace = 0
        self.position_trace = 0
        self.min = 0
        self.max = 0
        self.last_pos = 0
        self.daemon_publisher = rospy.Publisher('tidbd_killer_horde', daemon_killer, queue_size=10)
        self.age_threshold = 1000
        self.memory_size = 2**10
        self.num_tilings = 10

    def construct_obs(self, data):
        self.vel_trace = data.position_2 - self.last_pos + self.vel_trace * 0.8
        self.position_trace = data.position_2 + self.position_trace * 0.8
        self.last_pos = data.position_2
        data = np.array([
            data.position_2,
            self.position_trace,
            self.vel_trace,
        ])
        if self.min is not None and self.max is not None:
            self.min = np.minimum(self.min, data)
            self.max = np.maximum(self.max, data)
            data = (data + np.abs(self.min)) / (np.abs(self.min) + self.max)
            data = np.nan_to_num(data)
        else:
            self.min = data
            self.max = data
            data = np.ones(len(data))

        phi_next = np.zeros(self.memory_size)
        np.put(
            phi_next,
            np.concatenate(
                np.array(getTiles(
                    numtilings=self.num_tilings,  # the number of tilings in your tilecoder
                    memctable=self.memory_size - 1,  # the amount of memory for each tilecoder
                    floats=data * 10  # the observations from the robot
                )
                )
                ,
                int(2 ** 10)
            ),
            [1]
        )
        return phi_next

    def update(self, data):
        obs = self.construct_obs(data)
        [learner.update(obs, data) for learner in self.predictors]
        self.daemon_publisher.publish(
            self.fetch_alpha(),
            self.calc_alpha()
        )
        self.kill()

    def kill(self):
        alphas = self.fetch_alpha()
        mean_rupees = self.calc_alpha()
        np.put(mean_rupees, np.where(mean_rupees == 0), 1)
        np.nan_to_num(alphas)
        np.nan_to_num(mean_rupees)
        kill = np.array(mean_rupees).argsort()[:10]                                             # get the 10 worst
        step_size = alphas[int(len(alphas)/2)]/10                                               # new starting vals
        for i in kill:
            gamma = np.random.rand()
            lmbda = np.random.rand()
            reward_function = np.random.randint(len(reward_functions))
            #reward_function = np.random.randint(high=len(reward_functions))
            self.predictors[i] =\
                OnPolicyGVF(
                    step_size,
                    lmbda,
                    TDBDR(
                        number_of_features=2**10,
                        step_size=step_size,
                        meta_step_size=0.015,
                        active_features=10
                    ),
                    reward_functions[reward_function],
                    gamma,
                    constant,
                    name="_{0}".format(i)
                )

    def fetch_alpha(self):
        return np.array([np.sum(np.exp(daemon.learner.beta)) for daemon in self.predictors])

    def calc_alpha(self):
        a = self.fetch_alpha()
        return a / np.sum(a)


def listener():
    horde = DaemonKiller()
    step_size = 0.3/10

    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), poisiton_2, 0.98, constant, name="_0"))
    horde.add_learner(
        learner=OffPolicyGVF(0.03, 0.9, GTDR(2 ** 10, 0.03, moving_left_1, 10), poisiton_2, 0.99, constant, name="_1"))
    horde.add_learner(
        learner=OffPolicyGVF(0.03, 0.9, GTDR(2 ** 10, 0.03, moving_right_1, 10), poisiton_2, 0.5, constant, name="_2"))
    horde.add_learner(
        learner=OffPolicyGVF(0.03, 0.9, GTDR(2 ** 10, 0.03, moving_left_2, 10), poisiton_2, 0.9, constant, name="_3"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), is_moving_2, 0.98, constant, name="_4"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), poisiton_2, 0.98, constant, name="_5"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), voltage_2, 0.98, constant, name="_6"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), temperature_2, 0.98, constant, name="_7"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), command, 0.98, constant, name="_8"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.6, TDR(2 ** 10, 0.03, 10), load_2, 0.98, constant, name="_9"))
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that multiple can subscribe to the same topic
    rospy.Subscriber('robot_observations', servo_state, horde.update)  # subscribes to chatter and calls the callback
    rospy.spin()  # keeps python from exiting until this node is stopped


def tidbd_listener():
    horde = TIDBDDaemonKiller()
    step_size = 0.1/10
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.9, TDBDR(number_of_features=2**10, step_size=step_size, meta_step_size=0.015, active_features=10), is_moving_2, 0.98, constant, name="_4"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.9, TDBDR(number_of_features=2**10, step_size=step_size, meta_step_size=0.015, active_features=10), poisiton_2, 0.98, constant, name="_5"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.9, TDBDR(number_of_features=2**10, step_size=step_size, meta_step_size=0.015, active_features=10), voltage_2, 0.98, constant, name="_6"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.9, TDBDR(number_of_features=2**10, step_size=step_size, meta_step_size=0.015, active_features=10), temperature_2, 0.98, constant, name="_7"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.9, TDBDR(number_of_features=2**10, step_size=step_size, meta_step_size=0.015, active_features=10), command, 0.98, constant, name="_8"))
    horde.add_learner(learner=OnPolicyGVF(step_size, 0.9, TDBDR(number_of_features=2**10, step_size=step_size, meta_step_size=0.015, active_features=10), load_2, 0.98, constant, name="_9"))
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that mu$
    rospy.Subscriber('robot_observations', servo_state, horde.update)  # subscr$
    rospy.spin()  # keeps python from exiting until this node is stopped


def random_listener():
    horde = TIDBDDaemonKiller()
    step_size = 0.1/10
    for i in range(100):
        gamma = np.random.rand()
        lmbda = np.random.rand()
        reward_function = np.random.randint(len(reward_functions))
        horde.add_learner(
            learner=OnPolicyGVF(
                step_size,
                lmbda,
                TDBDR(
                    number_of_features=2**10,
                    step_size=step_size,
                    meta_step_size=0.015,
                    active_features=10
                ),
                reward_functions[reward_function],
                gamma,
                constant,
                name="_{0}".format(i)
            )
        )
    rospy.init_node('on_policy_listener', anonymous=True)  # anon means that mu$
    rospy.Subscriber('robot_observations', servo_state, horde.update)  # subscr$
    rospy.spin()  # keeps python from exiting until this node is stopped

if __name__ == '__main__':
    #listener()
    #tidbd_listener()
    random_listener()
