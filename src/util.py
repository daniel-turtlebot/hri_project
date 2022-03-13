from random import randint 

"""
* Filename: util.py
* Student: Harsh Deshpande, hdeshpande@ucsd.edu; Daniel Beaglehole, dbeaglehole@ucsd.edu; Divyam Bapna, dbapna@ucsd.edu; Chao Chi Cheng, cccheng@ucsd.edu
* Project #6:  GameBoi
*
* Description: This is the util file for GameBoi Project. The launch file to start the GameBoi is 
               main.launch under the launch folder. This program aims to store a set of predefined
               value for all the files to use
*
*How to use:
* Build:
*   catkin build
*   source ~/catkin_ws/devel/setup.bash
* Usage:
*   roslaunch gameboi main.launch
* Requirement:
*   Make sure every python files permission is set properly
"""

class GAME:
    '''
    Game Mode ENUM
    '''
    @classmethod
    def get_game(cls, i):
        return [cls.VISUAL,cls.MOVEMENT][i]
    VISUAL = 0
    MOVEMENT = 1

class GENERAL_STATE:
    '''
    GENERAL STATE ENUM
    '''
    INITIAL = 0
    SEARCHING = 1
    MOVING_TO_HUMAN = 2
    ASKING_HUMAN = 3
    PLAYING_GAME = 4

class GAME_STATE:
    '''
    Game STATE ENUM
    '''
    NOT_PLAYING = 0
    QUESTION = 1
    ANSWER = 2
    SIGNALS = 3
    DECISION = 4

class FLAGS:
    '''
    Flags ENUM
    '''
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
    '''
    Signals ENUM
    '''
    def __init__(self):
        self.RELEASE_SIGNAL = None
        self.PREVIOUS_STATE = None


class COLOR2TAG:
    '''
    Set of colors and its corresponding apriltag id
    '''
    COLORTAGS = {"Human":0,"Red":1,"Green":2,"Orange":3,"Blue":4,"Yellow":6}
    COLORS = list(COLORTAGS.keys())
    COLORTAGS2 = {"Human":0,"Red":21,"Green":22,"Orange":23,"Blue":24,"Yellow":26}




class CORRECT:
    '''
    Set of correct ans reponse
    '''

    @classmethod
    def get_action(cls, i):
        return [cls.nothing,cls.action][i]
    @staticmethod
    def nothing():
        return None
    @staticmethod
    def action():
        phr = 'nice work' if randint(0, 1) == 0 else 'way to go'
        return [('movement','spinning'),('phrases',phr)]

class INCORRECT:
    '''
    Set of incorrect ans reponse
    '''

    @classmethod
    def get_action(cls, i):
        return [cls.nothing,cls.action][i]
    @staticmethod
    def nothing():
        return None
    @staticmethod
    def action():
        phr = 'better luck next time' if randint(0, 1) == 0 else 'not very good'
        return [('phrases',phr + ' BEE BEE BEE BEE BEE')]

#We Have A Total Of 4 actions * 2 games. 
class ACTIONS:
    '''
    Convert index to action and game tuples
    '''
    @staticmethod
    def get_game_action(i):
        game = None
        correct = None
        incorrect = None
        
        game = GAME.get_game(i // 4)
        i = i % 4
        correct = CORRECT.get_action(i//2)
        incorrect = INCORRECT.get_action(i%2)
        return (game,correct,incorrect)

    def get_game_action1(i):
        game = None
        correct = None
        incorrect = None
        
        game = i // 4
        i = i % 4
        correct = i//2
        incorrect = i%2
        return (game,correct,incorrect)



        

class SPEACH_STRING:
    '''
    Pre Define Speech String
    '''
    Greetings = ["Hi, I'm Game Boy! How are you doing?","Hello! How are you? Would you like to play a game with me?", "Hey there, could I interest you for a fun game?" ]
    Post_Greetings = ["Ya, Lets Play The Game"]

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