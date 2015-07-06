"""A re-work of Craig's verifier which is a re-work of my verifier."""
import math


def verifier(config, obs, reward_calculator):
    """ provided a config containing"""
    gamma = config['gamma']
    safe_horizon = int(math.floor(2*(1/(1-gamma))))    # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon            # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                   # for each time-step to our terminal
        if i % 20 == 0:
            j = i
            ret_i = 0
            if i % 1000 == 0:
                print("{i} of {n}".format(i=i, n=terminal))
            while j < len(obs):              # for all the received rewards
                ret_i += reward_calculator.get_reward(obs[j]) * (gamma**(j-i))  # calculate our return
                j += 1
            ret.append(ret_i)                          # append the val