from src import learning_utils 

class Action:
    


class GameBot:
    def __init__(self):
        self.num_games = 3
        self.num_feedback = 6
        self.num_actions = self.num_games * self.num_feedback
        self.game = None # code in 0,1,2
        self.feedback = None # code in 0,1,...,N=number of types of feedback

        self.sampler = learning_utils.Sampler(self.num_actions)

        self.testing = True

    def do_action(self,action_idx):
        """
        Executes the action indexed by the argument. 
        Returns the reward from that action (perceived enjoyment)
        """
        pass

    def run(self):
        """
        Run experiments
        """

        # sample an action
        a_t = sampler.sample()

        # perform the action
        reward = do_action(a_t)

        # update sampling table
        sampler.update_index(reward,a_t)
         

if __name__ == "main":
    gamebot = GameBot()

    # If given a start signal 
    gamebot.run()

