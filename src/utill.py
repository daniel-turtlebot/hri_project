class GENERAL_STATE:
    INITIAL = 0
    SEARCHING = 1
    MOVING_TO_HUMAN = 2
    ASKING_HUMAN = 3
    PLAYING_GAME = 4

class GAME_STATE:
    NOT_PLAYING = 0
    QUESTION = 1
    ANSWER = 2
    SIGNALS = 3
    DECISION = 4

class FLAGS:
    def __init__(self):
        self.set()
    
    def set(self, start=None,human_found=None,human_reach=None,human_reponse=None,finish_game_q=None,
            finish_ans_verify=None,finish_game=None):
            self.START = start
            self.HUMAN_FOUND = human_found
            self.HUMAN_REACH = human_reach
            self.HUMAN_RESPONSE = human_reponse
            self.FINISH_GAME_Q = finish_game_q
            self.FINISH_ANS_VERIFY = finish_ans_verify
            self.FINISH_GAME = finish_game

class SIGNALS:
    def __init__(self):
        self.RELEASE_SIGNAL = None
        self.PREVIOUS_STATE = None