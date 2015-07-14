"""
A collection of methods which post-hoc calculate return for a particular experiment file

"""
import math
import pickle
from matplotlib import pyplot
import numpy as np

def calculate_discounted_return_backwards(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.

        What your should be using
    """
    gamma = config['gamma']                                 # find our gamma for convenience
    ret = np.zeros(len(obs))                                # initialize the array we hold our values in
    i = len(obs)-1                                          # starting at the end, heading backwards
    while i >= 0:                                           # until we reach the start of the array
        ret[i] += reward_calculator.get_reward(obs[i])      # add the
        try:                                                # we surround in a try catch in case we are at the start
            ret[i] += ret[i+1] * gamma                      # we add the previous return with a decay
        except:                                             # if there was no ret[i+1]
            pass                                            # should only occur for first element
        i -= 1                                              # move to the next element
    return ret[:len(ret)-1000]                              # slice the last 1000 elements


def calculate_discounted_return(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    safe_horizon = int(math.floor(2*(1/(1-gamma))))     # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon                    # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                           # for each time-step to our terminal
        j = i                                           # inner loop start index
        ret_i = 0                                       # set the value for the current time step's return

        if i % 1000 == 0 and i > 0:                     # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j < len(obs):                             # for all the received rewards from i onwards
            # add the reward of a time step discounted by gamma relative to where it is in relation to i
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma**(j-i))
            j += 1

        ret.append(ret_i)                               # append the calculated value
    return ret                                          # the calculated return for all time steps within safe range


def calculate_discounted_return(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    np.array
    gamma = config['gamma']
    safe_horizon = int(math.floor(2*(1/(1-gamma))))     # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon                    # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                           # for each time-step to our terminal
        j = i                                           # inner loop start index
        ret_i = 0                                       # set the value for the current time step's return

        if i % 1000 == 0 and i > 0:                     # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j < len(obs):                             # for all the received rewards from i onwards
            # add the reward of a time step discounted by gamma relative to where it is in relation to i
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma**(j-i))
            j += 1

        ret.append(ret_i)                               # append the calculated value
    return ret                                          # the calculated return for all time steps within safe range


def calculate_discounted_return_horizon(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    horizon = int(math.floor(1/(1-gamma)))      # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-(2 * horizon)           # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                   # for each time-step to our terminal
        j = i                                   # inner loop start index
        ret_i = 0                               # set the value for the current time step's return

        if i % 1000 == 0 and i > 0:             # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j < i+horizon:                    # only calculate a horizon's length away.
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma**(j-i))  # calculate our return
            j += 1

        ret.append(ret_i)                       # append the calculated val
    return ret                                  # the calculated return for all time steps within safe range


def calculate_return_horizon(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    horizon = int(math.floor(1/(1-gamma)))              # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-2*horizon                       # we only compute the return to a specific safe point
    rewards = []                                        # where each time-step's reward is stored
    ret = []                                            # the list of return values for each time-step

    for i in obs:                                       # this isn't needed, as we're not using sum
        rewards.append(reward_calculator.get_reward(i))

    for i in range(terminal):                           # for each time-step to our terminal
        j = i                                           # start index of inner loop
        ret_i = 0                                       # initialization for current time-step's return

        if i % 1000 == 0 and i > 0:                     # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        while j <= i + horizon:                         # from i to the horizon
            ret_i += rewards[j]                         # sum the value of the rewards
            j += 1

        ret.append(ret_i)
    return ret                                          # return the list of cumulative reward from each time-step


def calculate_return_total(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    safe_horizon = int(math.floor(2*(1/(1-gamma))))     # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon                    # we only compute the return to a specific safe point
    rewards = []
    ret = []
    for i in obs:                                       # extract all the rewards so we can use sum()
        rewards.append(reward_calculator.get_reward(i))

    for i in range(terminal):                           # for each time-step to our terminal

        if i % 1000 == 0 and i > 0:                     # pretty print
            print("{i} of {n}".format(i=i, n=terminal))

        ret.append(sum(rewards))                        # sum all the values
        rewards.pop()                                   # remove the head so that we progress to the next time-step
    return ret
