"""A re-work of Craig's verifier which is a re-work of my verifier."""
import math
import pickle
from matplotlib import pyplot


def calculate_discounted_return(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    safe_horizon = int(math.floor(2*(1/(1-gamma))))    # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon            # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                   # for each time-step to our terminal
        if i % 14 == 0:
            j = i
            ret_i = 0
            if i % 1000 == 0 and i > 0:
                print("{i} of {n}".format(i=i, n=terminal))
            while j < len(obs):              # for all the received rewards
                ret_i += reward_calculator.get_reward(obs[j]) * (gamma**(j-i))  # calculate our return
                j += 1
            ret.append(ret_i)                          # append the val
    pickle.dump(ret, open("verifier", "wb"))
    pyplot.plot(ret)
    pyplot.show()


def calculate_discounted_return_horizon(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the discounted return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    horizon = int(math.floor(1/(1-gamma)))    # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-(2 * horizon)            # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                   # for each time-step to our terminal
        j = i
        ret_i = 0
        if i % 1000 == 0 and i > 0:
            print("{i} of {n}".format(i=i, n=terminal))
        while j < i+horizon:              # only calculate a horizon's length away.
            ret_i += reward_calculator.get_reward(obs[j]) * (gamma**(j-i))  # calculate our return
            j += 1
        ret.append(ret_i)                          # append the val
    pickle.dump(ret, open("verifier", "wb"))
    pyplot.plot(ret)
    pyplot.show()


def calculate_return_horizon(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    horizon = int(math.floor(1/(1-gamma)))    # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-2*horizon            # we only compute the return to a specific safe point
    rewards = []
    ret = []
    for i in obs:
        rewards.append(reward_calculator.get_reward(i))

    for i in range(terminal):                   # for each time-step to our terminal
        j = i
        ret_i = 0
        if i % 1000 == 0 and i > 0:
            print("{i} of {n}".format(i=i, n=terminal))
        while j <= i + horizon:
            ret_i += rewards[j]
            j += 1
        ret.append(ret_i)
    pickle.dump(ret, open("verifier", "wb"))
    pyplot.plot(ret)
    pyplot.show()



def calculate_return_total(config, obs, reward_calculator):
    """
        provided a config containing a dictionary with the experiments
        gamma, all the observations, and some class with a reward calc,
        finds the return for all states within a safe range
        of the end of perception.
    """
    gamma = config['gamma']
    safe_horizon = int(math.floor(2*(1/(1-gamma))))    # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon            # we only compute the return to a specific safe point
    rewards = []
    ret = []
    for i in obs:
        rewards.append(reward_calculator.get_reward(i))

    for i in range(terminal):                   # for each time-step to our terminal
        if i % 1000 == 0 and i > 0:
            print("{i} of {n}".format(i=i, n=terminal))
        ret.append(sum(rewards))
        rewards.pop()
        # print("{i} is value {ret_i}".format(i=i, ret_i=ret_i))
    pickle.dump(ret, open("verifier", "wb"))
    pyplot.plot(ret)
    pyplot.show()
