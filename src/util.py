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
    COLORTAGS = {"Human":0,"Red":1,"Green":2,"Orange":3,"Blue":4,"Yellow":6}
    COLORS = list(COLORTAGS.keys())
    COLORTAGS2 = {"Human":0,"Red":21,"Green":22,"Orange":23,"Blue":24,"Yellow":26}




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

    def get_game_action1(i):
        game = None
        correct = None
        incorrect = None
        
        game = i // 16
        i = i % 16
        correct = i//4
        incorrect = i%4
        return (game,correct,incorrect)
        

class SPEACH_STRING:
    Greetings = ["Hello! How are you? Would you like to play a game with me?", "Hi, I'm Game Boy! How are you doing?", "Hey there, could I interest you for a fun game?" ]

    ## Game 1
    Instructions1 = ["I will show you a sequence of colors and if you can show me those colors in the right order you win."]

    Positive_Responses = ["Good Job", "Wow! You are good at this."]
    Negative_Responses = ["Sorry, looks like you picked the wrong color. Please try again.", "I'm afraid that is an incorrect sequence. Would you like to try again?"]
    Final_Responses = ["Wonderful! You have passed the game.", "I wish I had a prize for you. How about a high five?", "Amazing! You won."]

## Game 2
    Instructions2 = ["I'll show you around while I look for those colors."]
    Start2 = ["Come, let's do this together.", "Let's go finish the sequence.", "It's fine if you want to wait. I won't be gone too long."]

    First_Color2 = ["We already found our first color."]
    Positive_Responses2 = ["We are doing great. You are an awesome guide.", "Look, we already found {} colors."]