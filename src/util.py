from random import randint 

class GAME:
    @classmethod
    def get_game(cls, i):
        return [cls.VISUAL,cls.MOVEMENT][i]
    VISUAL = 0
    MOVEMENT = 1

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


class COLOR2TAG:
    COLORTAGS = {"Human":0,"Pink":1,"Red":2,"Orange":3,"Green":4,"Blue":5,"Yellow":6}
    COLORS = list(COLORTAGS.keys())




class CORRECT:
    
    @classmethod
    def get_action(cls, i):
        return [cls.nothing,cls.phrases,cls.sound,cls.movement][i]
    @staticmethod
    def nothing():
        return None
    @staticmethod
    def phrases():
        phr = 'nice work' if randint(0, 1) == 0 else 'way to go'
        return ('phrases',phr) 
    @staticmethod
    def sound():
        return ('sound','positive')
    @staticmethod
    def movement():
        return ('movement','spinning')

class INCORRECT:
    @classmethod
    def get_action(cls, i):
        return [cls.nothing,cls.phrases,cls.sound,cls.negative_phrases][i]
    @staticmethod
    def nothing():
        return None
    @staticmethod
    def phrases():
        phr = 'you can do it' if randint(0, 1) == 0 else 'you got this'
        return ('phrases',phr)
    @staticmethod
    def sound():
        return ('sound','negative')
    def negative_phrases():
        phr = 'better luck next time' if randint(0, 1) == 0 else 'not very good'
        return ('phrases',phr)

#We Have A Total Of 16 actions * 2 games. 
class ACTIONS:
    @staticmethod
    def get_game_action(i):
        game = None
        correct = None
        incorrect = None
        
        game = GAME.get_game(i // 16)
        i = i % 16
        correct = CORRECT.get_action(i//4)
        incorrect = INCORRECT.get_action(i%4)
        return (game,correct,incorrect)
        



