"""
    We define a generic Experiment to be run in lieu of a MDP Problem.
    Handles all of the data which we're passing back and forth. Any
    feature-processing, changing of gamma, .etc should be done here.
    You can simply extend this template and over-ride as necessary.
"""

# from pysrc.utilities.verifier import Verifier

__author__ = 'alex'


class Experiment(object):
    """" Analagous to the mdp,  """

    def __init__(self, config):
        self.starting_element = 0
        self.num_tilings = 1
        self.memory_size = 1
        9
        self.gamma = 0.01

    def step(self):
        """ Given a set of features from a file loader, processes the observations
               into a set of features compatible with the problem we're trying to
               achieve.
        """

        # no a

        # next state
        # noise?
        # R = Reward
        # g = gamma
        # gnext = gamma next
        # next state
        # parameters = {'l':config['lambda'],
        #           'lnext':config['lambda'],
        #           'g':config['gamma'],
        #           'gnext':config['gamma']}  # we presume that for the time being, these don't change
        # phinext and phi



        """
          def step(self):
    a = 0 if self.na==1 else self.getAction(self.bpol, self.s, self.rdrun)
    snext   = self.getNextState(self.Pssa, self.s, a, self.rdrun)
    noise   = self.rdrun.normal(0, self.Rstd) if self.Rstd>0 else 0.
    R       = self.getReward(self.Rssa, self.s, snext, a) + noise
    phi     = self.Phi[self.s]
    phinext = self.Phi[snext]
    g       = self.Gamma[self.s, self.s]
    gnext   = self.Gamma[snext, snext]
    stemp   = self.s
    self.s  = snext
    return {'s':stemp, 'phi':phi, 'act':a, \
            'R':R, 'snext':snext, 'phinext':phinext, \
            'g':g, 'gnext':gnext}
        """