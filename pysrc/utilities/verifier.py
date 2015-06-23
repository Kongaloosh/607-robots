import numpy as np

class Verifier(object):

    def __init__(self, gamma, instantaneous=False):
        self.instantaneous = instantaneous
        self.horizon = self.timesteps_from_gamma(gamma)
        
        # using numpy instead of deque as we need to do calculations on the data rather
        # than iterate of the individual items
        if not self.instantaneous:
            self.reward_history = np.zeros(self.horizon)
        else:
            self.reward_history = None
            
        self.prediction_history = np.zeros(self.horizon)
        
        self.count = 0

    def _calculate_reward(self):
        ret_val = None
        if self.instantaneous:
            ret_val = self.reward_history
        else:
            # in the cumulative 
            ret_val = np.sum(self.reward_history)
            
        return ret_val
        
    def update(self, reward, prediction):
        if self.instantaneous:
            self.reward_history = reward
        else:
            self.append_to_np_array(self.reward_history, reward)
        
        ret_val = None
        
        if self.count == self.horizon:
            ret_val = self._calculate_reward() - self.prediction_history[0]
            
        self.append_to_np_array(self.prediction_history, prediction)
        
        self.count = min(self.count + 1, self.horizon)
            
        return ret_val
            
    def timesteps_from_gamma(self, gamma):
        return 1/(1-gamma)

    @staticmethod
    def append_to_np_array(array, val):
        array[0:-1] = array[1:]
        array[-1] = val