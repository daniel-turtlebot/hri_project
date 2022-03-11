import os
import numpy as np
from src import learning_utils 
import matplotlib.pyplot as plt
from sklearn.linear_model import LogisticRegression

class GameBot:
    def __init__(self):
        self.num_games = 3
        self.num_feedback = 6
        self.num_actions = self.num_games * self.num_feedback
        self.game = None # code in 0,1,2
        self.feedback = None # code in 0,1,...,N=number of types of feedback

        self.sampler = learning_utils.Sampler(self.num_actions)

        self.testing = True
        self.t = 0
        self.X = [] 
        self.y = [] 

    def do_action(self,action_idx):
        """
        Executes the action indexed by the argument. 
        Returns the reward from that action (perceived enjoyment)
        """
        pass
    
    def action_to_x(self,a_t):
        """ 
        Convert action index to a single vector of categorical vars
        """
        pass

    def run(self):
        """
        Run experiments
        """

        ## sample an action
        a_t = sampler.sample()

        ## perform the action
        y_t = do_action(a_t) # return reward

        ## update sampling table
        sampler.update_index(y_t,a_t)

        x_t = self.action_to_x(a_t)
        self.t += 1
        self.X.append(x_t)
        self.y.append(y_t)
         

if __name__ == "main":
    gamebot = GameBot()

    # If given a start signal 
    gamebot.run()

    Xs = gamebot.X
    ys = gamebot.y

    model = LogisticRegression(solver='liblinear', random_state=0)
    model.fit(Xs, ys)

    cwd = os.getcwd()
    data_file = cwd + "/estimator.txt"
    params_file = cwd + "/params.txt"

    np.savetxt(data_file,Xs)
    np.savetxt(data_file,ys)
    np.savetxt(params_file,model.intercept_)
    np.savetxt(params_file,model.coef_)



