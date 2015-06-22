import numpy

class Verifier(object):
    
    def __init__(self , rlGamma):
        if rlGamma > 1 or rlGamma < 0:
            raise"Gamma is an ivalid value. Is is between 0 and 1? "

        self.rlGamma = rlGamma
        self.rewardHistory = list()
        self.predictionHistory = list()
        # (int) (Math.log(precision) / Math.log(gamma));

    def calculateReturn(self):
        if len(self.rewardHistory) >= int(self.horizon):
            returnValue = 0;
            for idx, val in enumerate(reversed(self.rewardHistory)):
                idx = len(self.rewardHistory)-idx-1 #true idx
                returnValue += self.rewardHistory[idx] * (self.rlGamma)**(((len(self.rewardHistory) -1))-idx)
            return returnValue
        return None
        
    def updateReward(self, reward):
        self.rewardHistory.append(reward)
        length = len(self.rewardHistory)
        if length > int(self.horizon):
            self.rewardHistory.pop(0) 
            
    def updatePrediction(self, prediction):
        self.predictionHistory.append(prediction)
        if len(self.predictionHistory) > int(self.horizon):
            self.predictionHistory.pop(0)

    def update(self, reward, prediction):
        self.updateReward(reward)
        self.updatePrediction(prediction)
            
    def getSyncedPrediction(self):
        return self.predictionHistory[0]

    
    def calculateCurrentError(self):
        return_calced = self.calculateReturn()
        if return_calced == None: return None
        return abs(return_calced - self.predictionHistory[0])