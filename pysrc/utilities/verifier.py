"""A re-work of Craig's verifier which is a re-work of my verifier."""


def verifier(config, obs, reward):
    """ provided a config containing"""
    gamma = config['gamma']
    safe_horizon = 2*(1/(1-gamma))    # abstract out the '2' to some config (experiment?)
    terminal = len(obs)-safe_horizon            # we only compute the return to a specific safe point
    ret = []
    for i in range(terminal):                   # for each time-step to our terminal
        ret_i = 0
        for idx, s in enumerate(obs):              # for all the received rewards
            ret_i = s[reward] * (gamma**idx)       # calculate our return
        ret.append(ret_i)                          # append the val