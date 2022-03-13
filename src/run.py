import os
import numpy as np
import learning_utils 
import util
import matplotlib.pyplot as plt
from sklearn.linear_model import LogisticRegression
from random import randint 


def my_mean(tup):
    total = 0
    for i,val in enumerate(list(tup)):
        if i == 0:
            total += val
        else:
            total += 6-val
    return total/len(tup)

class GameBot:
    def __init__(self):
        self.num_games = 2
        
        self.num_correct = 2
        self.num_incorrect = 2
        self.num_feedback = self.num_correct * self.num_incorrect
        self.num_actions = self.num_games * self.num_feedback
        
        self.dir_path = "/home/turtlebot/chocolate_ws/src/gameboi/survey/"
        self.sampler = learning_utils.Sampler(self.num_actions)

        self.testing = True
        self.t = 0
        self.X = [] 
        self.y = [] 

        self.a_t = None

    def action_to_x(self,a_t):
        """ 
        Convert action index to a single vector of one-hot encoded vars
        """
        x_dim = self.num_games + self.num_correct + self.num_incorrect
        x = np.zeros(x_dim)
        game,correct,incorrect = util.ACTIONS.get_game_action1(a_t)
        x[game] = 1
        x[self.num_games + correct] = 1
        x[self.num_games + self.num_correct + incorrect] = 1 
        return x

    
    def get_action(self):
        """
        Call From Main Controller, replace first part of run and do_action
        """
        ## sample an action & perform the action
        self.a_t = self.sampler.sample()
        return self.a_t
    
    def update_reward(self, pre_survey, reward_survey, reward_emotion,num_fails):
        """
        Call From Main Controller, replace second part of run
        pre_survey - tuple of size 3 with entries 1-5
        reward_survey, reward_emotion - int from 1-5
        """
        max_survey_val = 5
        normalizer = my_mean(pre_survey)/max_survey_val

        # between 0 and 1
        y_t = (normalizer*reward_survey * 0.9 + reward_emotion * 0.1)/max_survey_val 

        ## update sampling table
        self.sampler.update_index(y_t, self.a_t)
        x_t = self.action_to_x(self.a_t)
        self.t += 1
        self.X.append(x_t)
        self.y.append(y_t)
        self.a_t = None

        np.savetxt(self.dir_path + "S.txt",self.sampler.S) # save S
        np.savetxt(self.dir_path + "F.txt",self.sampler.F) # save F

        Xfile = "X.txt"
        yfile = "y.txt"

        np.savetxt(self.dir_path + Xfile,self.X)
        np.savetxt(self.dir_path + yfile,self.y)

    def do_action(self):
        pass

    def end(self):
        Xs = self.X
        ys = self.y

        model = LogisticRegression(solver='liblinear', random_state=0)
        model.fit(Xs, ys)

        params_file = "/home/turtlebot/chocolate_ws/src/gameboi/survey/params.txt"
        data_file = "/home/turtlebot/chocolate_ws/src/gameboi/survey/estimator.txt"

        np.savetxt(data_file,Xs)
        np.savetxt(data_file,ys)
        np.savetxt(params_file,model.intercept_)
        np.savetxt(params_file,model.coef_)


    def run(self):
        ### Run experiments

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



