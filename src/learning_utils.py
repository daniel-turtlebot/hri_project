from numpy import random 
import numpy as np

class Sampler:
    def __init__(self, K):
        self.t = None
        self.S = np.zeros(K)
        #what is this?
        self.F = np.zeros(K)

    def sample(self):
        theta_t = random.beta(self.S,self.F)
        return np.argmax(theta_t)

    def update_index(self,r_t,arm):
        s_t = random.binomial(1,r_t) 
        if s_t == 1:
            self.S[arm] += 1
        else:
            self.F[arm] += 1


    
