"""
A collection of methods which post-hoc calculate return for a particular experiment file

"""
import math
import numpy as np


class RUPEE(object):

    def __init__(self, number_of_features, beta, alpha):
        self.h = np.zeros(number_of_features)
        self.beta = beta
        self.alpha = alpha
        self.tao = 0
        self.delta_e = 0

    def update(self, delta, e, phi):
        self.tao = (1 - self.beta) * self.tao + self.beta
        self.beta /= self.tao
        self.delta_e = (1-self.beta)*self.delta_e + self.beta * e * delta
        self.h += self.alpha * (delta * e - np.dot(self.h, phi) * phi)
        return np.sqrt(np.abs(np.dot(self.h, self.delta_e))**self.beta)


class UDE(object):

    def __init__(self, beta):
        self.delta_bar = 0
        self.beta = beta
        self.mean = 0
        self.time = 1
        self.variance = 0


    def update(self, delta):
        self.delta_bar = delta * self.beta + self.delta_bar * (1 - self.beta)
        self.mean += (delta-self.mean)/self.time
        self.variance += (delta - self.mean)**2 / self.time
        self.time += 1
        return np.abs(self.delta_bar / (np.sqrt(self.variance) + 0.001))


class OnlineVerifier(object):
    """Calculates the true return"""

    def __init__(self, rlGamma):
        self.rlGamma = list()# todo: make gamma a function
        self.rewardHistory = list()  # a list of rewards we use to calculate true return
        self.predictionHistory = list()  # a list of predictions to sync our error calculation
        precision = 0.01
        self.horizon = (np.log(precision) / np.log(rlGamma))  # the horizon which we use to determine precision

    def calculate_currente_return(self):
        """calculate given the most recent reward, the true return"""
        # todo: go over convoluted old code
        if len(self.rewardHistory) >= int(self.horizon):
            returnValue = 0
            for idx, val in enumerate(reversed(self.rewardHistory)):
                idx = len(self.rewardHistory) - idx - 1  # true idx
                if idx == 0:
                    returnValue += self.rewardHistory[idx]
                else:
                # returnValue += self.rewardHistory[idx] * (self.rlGamma[idx]) ** (((len(self.rewardHistory) - 1)) - idx)
                    returnValue += self.rewardHistory[idx] * reduce(lambda x, y: x*y, reversed(self.rlGamma[idx-1:]))
            return returnValue
        return None

    def update_gamma(self, gamma):
        self.rlGamma.append(gamma)
        if len(self.rlGamma) > int(self.horizon):
            self.rlGamma.pop(0)

    def update_reward(self, reward):
        """Adds the new reward"""
        self.rewardHistory.append(reward)
        length = len(self.rewardHistory)
        if length > int(self.horizon):
            self.rewardHistory.pop(0)

    def update_prediction(self, prediction):
        """"""
        self.predictionHistory.append(prediction)
        if len(self.predictionHistory) > int(self.horizon):  # if the buffer is full, pop the first off.
            self.predictionHistory.pop(0)

    def synced_prediction(self):
        """Gets the prediction which matches the current return estimate"""
        return self.predictionHistory[0]

    def calculate_current_error(self):
        """Gets the difference between the calculated return and the prediction"""
        return self.calculate_currente_return() - self.synced_prediction()

    def update_all(self, gamma, reward, prediction):
        self.update_gamma(gamma)
        self.update_reward(reward)
        self.update_prediction(prediction)

def calculate_discounted_return_backwards(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.

        What your should be using
    """
    gamma = config['gamma']  # find our gamma for convenience
    ret = np.zeros(len(obs))  # initialize the array we hold our values in
    i = len(obs) - 1  # starting at the end, heading backwards
    while i >= 0:  # until we reach the start of the array
        ret[i] += reward_calculator.get_reward(obs[i])  # add the
        try:  # we surround in a try catch in case we are at the start
            ret[i] += (ret[i + 1] * gamma)  # we add the previous return with a decay
        except:  # if there was no ret[i+1]
            pass  # should only occur for first element
        i -= 1  # move to the next element
    return ret[:len(ret) - 1000]  # slice the last 1000 elements


def calculate_discounted_return(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    safe_horizon = int(math.floor(2 * (1 / (1 - gamma))))  # abstract out the '2' to some config (experiment?)
    terminal = len(obs) - safe_horizon  # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):  # for each time-step to our terminal
        j = i  # inner loop start index
        ret_i = 0  # set the value for the current time step's return

        if i % 1000 == 0 and i > 0:  # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j < len(obs):  # for all the received rewards from i onwards
            # add the reward of a time step discounted by gamma relative to where it is in relation to i
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma ** (j - i))
            j += 1

        ret.append(ret_i)  # append the calculated value
    return ret  # the calculated return for all time steps within safe range


def calculate_discounted_return(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    np.array
    gamma = config['gamma']
    safe_horizon = int(math.floor(2 * (1 / (1 - gamma))))  # abstract out the '2' to some config (experiment?)
    terminal = len(obs) - safe_horizon  # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):  # for each time-step to our terminal
        j = i  # inner loop start index
        ret_i = 0  # set the value for the current time step's return

        if i % 1000 == 0 and i > 0:  # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j < len(obs):  # for all the received rewards from i onwards
            # add the reward of a time step discounted by gamma relative to where it is in relation to i
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma ** (j - i))
            j += 1

        ret.append(ret_i)  # append the calculated value
    return ret  # the calculated return for all time steps within safe range


def calculate_discounted_return_horizon(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    horizon = int(math.floor(1 / (1 - gamma)))  # abstract out the '2' to some config (experiment?)
    terminal = len(obs) - (2 * horizon)  # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):  # for each time-step to our terminal
        j = i  # inner loop start index
        ret_i = 0  # set the value for the current time step's return

        if i % 1000 == 0 and i > 0:  # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j < i + horizon:  # only calculate a horizon's length away.
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma ** (j - i))  # calculate our return
            j += 1

        ret.append(ret_i)  # append the calculated val
    return ret  # the calculated return for all time steps within safe range


def calculate_return_horizon(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    horizon = int(math.floor(1 / (1 - gamma)))  # abstract out the '2' to some config (experiment?)
    terminal = len(obs) - 2 * horizon  # we only compute the return to a specific safe point
    rewards = []  # where each time-step's reward is stored
    ret = []  # the list of return values for each time-step

    for i in obs:  # this isn't needed, as we're not using sum
        rewards.append(reward_calculator.get_reward(i))

    for i in range(terminal):  # for each time-step to our terminal
        j = i  # start index of inner loop
        ret_i = 0  # initialization for current time-step's return

        if i % 1000 == 0 and i > 0:  # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j <= i + horizon:  # from i to the horizon
            ret_i += rewards[j]  # sum the value of the rewards
            j += 1

        ret.append(ret_i)
    return ret  # return the list of cumulative reward from each time-step


def calculate_return_total(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    safe_horizon = int(math.floor(2 * (1 / (1 - gamma))))  # abstract out the '2' to some config (experiment?)
    terminal = len(obs) - safe_horizon  # we only compute the return to a specific safe point
    rewards = []
    ret = []
    for i in obs:  # extract all the rewards so we can use sum()
        rewards.append(reward_calculator.get_reward(i))

    for i in range(terminal):  # for each time-step to our terminal

        if i % 1000 == 0 and i > 0:  # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        ret.append(sum(rewards))  # sum all the values
        rewards.pop()  # remove the head so that we progress to the next time-step
    return ret
