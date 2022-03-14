"""
* Filename: learning_utils.py
* Student: Harsh Deshpande, hdeshpande@ucsd.edu; Daniel Beaglehole, dbeaglehole@ucsd.edu; Divyam Bapna, dbapna@ucsd.edu; Chao Chi Cheng, cccheng@ucsd.edu
* Project #6:  GameBoi
*
* Description: This file contains the online learning utils for this project.  
              
             
*
*How to use: Create a Sampler object and call its update_index() method to 
incorporate rewards, and sample() method to select an updated strategy.
"""

from numpy import random 
import numpy as np
from os.path import exists

class Sampler:
    """
    Class for sampling according to the Thomson-Sampling method
    """
    def __init__(self, K):
        self.t = None
        dir_path = "/home/turtlebot/chocolate_ws/src/gameboi/survey/"
        if exists(dir_path + "S.txt"): # if S history exists
            self.S = np.loadtxt(dir_path + "S.txt")
        if exists(dir_path + "F.txt"): # if F history exists
            self.F = np.loadtxt(dir_path + "F.txt")
        else: # if no history
            self.S = np.ones(K)
            self.F = np.ones(K)

    def sample(self):
        theta_t = random.beta(self.S,self.F)
        return np.argmax(theta_t)

    def update_index(self,r_t,arm):
        s_t = random.binomial(1,r_t) 
        if s_t == 1:
            self.S[arm] += 1
        else:
            self.F[arm] += 1


    
