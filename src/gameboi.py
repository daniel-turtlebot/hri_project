#!/home/turtlebot/anaconda3/envs/gui/bin/python3

import gui_gameboi as guigb
from util import *
from run import GameBot

from random import randint 
import numpy as np
import threading
import signal
import time
import sys
import pyttsx3


import rospy
import wx

from std_msgs.msg import String
from sensor_msgs.msg import  Image, CompressedImage
from geometry_msgs.msg import Twist


"""
* Filename: gameboi.py
* Student: Harsh Deshpande, hdeshpande@ucsd.edu; Daniel Beaglehole, dbeaglehole@ucsd.edu; Divyam Bapna, dbapna@ucsd.edu; Chao Chi Cheng, cccheng@ucsd.edu
* Project #6:  GameBoi
*
* Description: This is the main control file for GameBoi Project. The launch file to start 
               the GameBoi is main.launch under the launch folder. This program aims to control 
               the turtle bot based on the current state (FSM) and the information it receives from 
               game mover, emotion detection, game checker and gui. The program will start a new game
               once a round is finished, so no need to restart. We have defined a set of values in 
               util.py so it can be accessed across the file.
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

SKIPGAME = True
DEBUG = True

def dprint(text):
    if DEBUG:
        print(text)


def sigint_handler(signal, frame):
    """
    Function to handle the terminal signal
    """
    dprint("STOP PROGRAM")
    gameboi.backend.end()
    if gameboi:
        if gameboi.gui:
            gameboi.gui.stop()
    sys.exit(0)





class GameBoi:
    """
    Main Class to run the controller program
    """

    def __init__(self):
        rospy.init_node('GameBoi')
       
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.game_check_pub = rospy.Publisher('/game_check_state', String, queue_size=1)
        self.game_check_sub = rospy.Subscriber('/game_checker', String , self.game_check_cb , queue_size=1)

        self.game_mover_pub = rospy.Publisher('/game_mover_state',String , queue_size=1)
        self.game_mover_sub = rospy.Subscriber('/game_mover',String, self.game_mover_cb, queue_size=1)
        

        self.emotion_pub = rospy.Publisher('/emotion_state', String, queue_size=1)
        self.emotion_sub = rospy.Subscriber('/emotion',String, self.emotion_cb, queue_size=1)
       
        # a set of state flags to indicate current situation
        self.flags = FLAGS()
        self.signals = SIGNALS()
        self.game_mode = GAME.VISUAL
        self.state = GENERAL_STATE.INITIAL
        self.game_state = GAME_STATE.NOT_PLAYING


        self.backend = GameBot()
        self.audio = pyttsx3.init()
        self.audio.setProperty('rate', self.audio.getProperty('rate') -50 )   

        self.reset()
        self.panel = None
        self.gui = None

        #Start a new thread to run gui
        self.t1 = threading.Thread(target=self.start_gui)
        self.t1.start()
        while self.panel is None:
            rospy.sleep(0.2)

    def reset(self):
        """
        Reset critical variables for the new game
        """
        self.game_check_str = None
        self.game_mover_str = None
        self.resp_incorrect = None
        self.resp_correct = None
        self.game_seq = None
        self.reward_survey = 0
        self.reward_emotion = None
        self.num_fails = 0

    def audio_out_helper(self,text):
        """
        Output the text to laptop sound system
        """
        self.audio.say(text)
        self.audio.runAndWait()

    def audio_out(self,text,anyc=False):
        """
        Output the text to laptop sound system
        Will randomly choose the text if it is a list
        Run in new thread if required, to prevent blocking
        """

        if type(text) == list:
            text = text[randint(0,len(text)-1)]

        if anyc:
            self.t2 = threading.Thread(target=self.audio_out_helper, args=(text,))
            self.t2.start()
        else:
            self.audio_out_helper(text)
        

        
    def start_gui(self):
        """
        Start Gui
        """
        self.gui = guigb.gui()
        dprint("GUI START")
        self.panel = self.gui.frame.panel
        self.gui.start()

    def get_game_action(self):
        """
        get the game and actions to do from our backend decision 
        """
        actions = ACTIONS.get_game_action(self.backend.get_action())
        self.game_mode, self.resp_correct, self.resp_incorrect = actions
        self.game_seq = self.generate_game_seq()

    def generate_game_seq(self):
        """
        Ramdomly generate game sequence
        Range from 3 to 5 with 5 different colors
        """
        colors = COLOR2TAG.COLORS
        colors_len = len(colors) - 1
        previos = ''
        seq = 'start'
        for _ in range(randint(3,5)):
            color = colors[randint(0,colors_len)]
            while color == 'Human' or color == previos:
                color = colors[randint(0,colors_len)]
            previos = color
            seq = f'{seq} {color}'
        return seq

    def do_action(self,action):
        """
        Apply action to robot based on action given
        """
        action = action()
        if action is None:
            return
        act_type, act = action
        if act_type == 'sound':
            if act == 'positive':
                self.audio_out('YA YA YA YA YA')
            else:
                self.audio_out('BEE BEE BEE BEE BEE')
        elif act_type == 'phrases':
            self.audio_out(act)
        elif act_type == 'movement':
            self.audio_out('You Are Correct YA',anyc=True)


            self.pub_bot_vel(0,.5,0.2,repeat=4)
            self.pub_bot_vel(0,-0.5,0.2,repeat=4)
            self.pub_bot_vel(0,.5,0.2,repeat=4)
            self.pub_bot_vel(0,-0.5,0.2,repeat=4)
            pass
            
    def start_game(self):
        """
        Start the game with human
        """
        
        self.reset()
        self.pre_survey()
        
        self.pub_emotion("start")
        self.update_gui_text("Let's Play The Game!")
        self.audio_out(SPEACH_STRING.Greetings)
        self.get_game_action()
        rospy.sleep(2)

    def update_gui_text(self,text):
        """
        Send the text to GUI
        """
        wx.CallAfter(self.panel.update_text, text)

    def game_mover_cb(self,s):
        """
        call back for receiving data from game mover
        """
        self.game_mover_str = s.data

    def game_check_cb(self, s):
        """
        call back for receiving data from game check
        """
        self.game_check_str = s.data

    def emotion_cb(self, s):
        """
        call back for receiving data from emotion detection
        """
        self.reward_emotion = float(s.data)

    def pub_mover(self,text):
        """
        publish data from game mover
        """
        s = String()
        s.data = text
        self.game_mover_pub.publish(s)

    def pub_checker(self,text):
        """
        publish data from game check
        """
        s = String()
        s.data = text
        self.game_check_pub.publish(s)

    def pub_emotion(self,text):
        """
        publish data from emotion detection
        """
        s = String()
        s.data = text
        self.emotion_pub.publish(s)
    
            
    def init_start(self):
        """
        Initial start to enter the new game
        """
        if self.panel.get_enter_flag():
            self.flags.set(start=True)
        self.stop()

    def search_for_target(self):
        """
        Find Human In The ROom
        """
        # self.pub_mover("start Human")
        # while(self.game_mover_str != 'end'):
        #     print("WAITING SEARCHING")
        #     rospy.sleep(0.5)
        self.flags.set(human_found=True)
        

    def prompt_Q(self):
        """
        Ask If Human Want To Play A Game
        """
        self.update_gui_text("Do You Want To Play A Game With Me?")
        want_to_play = False
        for _ in range(50):
            if self.panel.get_enter_flag():
                want_to_play = True
                break
            rospy.sleep(0.2)

        if want_to_play:
            self.start_game()
            self.flags.set(human_reponse=True)
        else:
            self.flags.set(human_reponse=False)


    def prompt_game_Q(self):
        """
        Ask  Human Game Question
        """
        # self.flags.set(finish_game_q=True)
        # return
        if SKIPGAME:
            self.flags.set(finish_ans_verify=True)
            return
        if self.game_mode == GAME.VISUAL:
            self.update_gui_text(self.game_seq)
            self.audio_out(SPEACH_STRING.Instructions1,anyc=True)
            rospy.sleep(6)
        else:
            self.update_gui_text("Please Follow The Robot And Remember The Color")
            self.audio_out(SPEACH_STRING.Instructions2)
            self.pub_mover(self.game_seq)
            while(self.game_mover_str != 'end'):
                dprint("WAITING Q")
                rospy.sleep(0.5)

        self.flags.set(finish_game_q=True)

    def verify_ans(self):
        """
        Check Human Answer
        """
        if SKIPGAME:
            self.flags.set(finish_ans_verify=True)
            return
        self.update_gui_text("Please Display The Colors In Order")
        rospy.sleep(1)
        self.pub_checker(self.game_seq)
        while(not self.game_check_str):
            dprint("FIND STRING")
            rospy.sleep(0.5)
        ps = ''
        while(not 'Passed' in self.game_check_str ):
            if ps != self.game_check_str:
                self.update_gui_text(self.game_check_str)
                ps = self.game_check_str
                if 'Wrong' in ps:
                    self.num_fails += 1
                if not 'LOOKING' in ps:
                    self.do_action(self.resp_incorrect if 'Wrong' in ps else self.resp_correct)
                    

            rospy.sleep(1)

        self.update_gui_text(self.game_check_str)
        self.audio_out(SPEACH_STRING.Final_Responses,anyc=True)
        rospy.sleep(1)

        self.game_check_str = None
        self.flags.set(finish_ans_verify=True)

    def pre_survey(self):
        """
        Pop up survey before the game
        """
        wx.CallAfter(self.gui.frame.show_pre_form)

        while self.gui.frame.form_panel is None:
            rospy.sleep(0.1)
        
        while not self.gui.frame.form_panel.button_clicked:
            rospy.sleep(0.5)
        
       
        try:
            self.pre_reward = tuple([ float(text) if text else 1 for text in self.gui.frame.form_panel.saved_rating])
        except:
            dprint("Conversion Error 1")
            self.pre_reward = (1,1,1)
        
        wx.CallAfter(self.gui.frame.show_game)
        
        self.panel = self.gui.frame.panel

    def update_backend(self):
        """
        Pop up survey After the game, and send the value to the backend to update decision
        """

        wx.CallAfter(self.gui.frame.show_post_form)
        self.pub_emotion("end")

        while self.gui.frame.form_panel is None:
            rospy.sleep(0.1)
        while not self.gui.frame.form_panel.button_clicked:
            rospy.sleep(0.5)

        while(self.reward_emotion is None):
            dprint("Waiting Reward EMO")
            rospy.sleep(0.5)

        dprint("TEST")
        if self.gui.frame.form_panel.saved_rating:
            try:
                self.post_reward = float(self.gui.frame.form_panel.saved_rating)
            except:
                dprint("Conversion Error 2")
                self.post_reward = 1
        else:
            self.post_reward = 1
        self.reward_emotion = 5 * (self.reward_emotion/100)
        self.backend.update_reward(self.pre_reward,self.post_reward,self.reward_emotion,self.num_fails)
        self.flags.set(finish_game=True)

        fileh = open('/home/turtlebot/chocolate_ws/src/gameboi/survey/survey.txt',"a")
        fileh.write(f'time {time.time()}, emo {self.reward_emotion}, pre {self.pre_reward}, post {self.post_reward} , fails {self.num_fails}')
        fileh.close()
        dprint("TEST1")
        wx.CallAfter(self.gui.frame.show_game)
        self.panel = self.gui.frame.panel
        self.update_gui_text("Press Enter To Restart The Game")

    def stop(self):
        """
        Stop The Turtlebot Movement
        """
        self.pub_bot_vel(0, 0, 0, repeat=1)

    def pub_bot_vel(self,linear,angular,time,repeat=1):
        """
        Move TurtleBot Base On Given Values
        """
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        for _ in range(repeat):
            self.velocity_pub.publish(t)
            rospy.sleep(time)

    def change_state(self):
        """
        Change The Game States
        """
        if self.state == GENERAL_STATE.INITIAL:
            if self.flags.START:
                self.state = GENERAL_STATE.SEARCHING
        elif self.state == GENERAL_STATE.SEARCHING:
            if self.flags.HUMAN_FOUND:
                self.state = GENERAL_STATE.MOVING_TO_HUMAN
        elif self.state == GENERAL_STATE.MOVING_TO_HUMAN:
            if self.flags.HUMAN_REACH:
                self.state = GENERAL_STATE.ASKING_HUMAN
        elif self.state == GENERAL_STATE.ASKING_HUMAN:
            if self.flags.HUMAN_RESPONSE == True:
                self.state = GENERAL_STATE.PLAYING_GAME
            elif self.flags.HUMAN_RESPONSE == False:
                self.state = GENERAL_STATE.SEARCHING
        elif self.state == GENERAL_STATE.PLAYING_GAME:
            if self.game_state == GAME_STATE.NOT_PLAYING:
                self.game_state = GAME_STATE.QUESTION
            elif self.game_state == GAME_STATE.QUESTION:
                if self.flags.FINISH_GAME_Q:
                    self.game_state = GAME_STATE.ANSWER
            elif self.game_state == GAME_STATE.ANSWER:
                if self.flags.FINISH_ANS_VERIFY:
                    self.game_state = GAME_STATE.DECISION
            elif self.game_state == GAME_STATE.SIGNALS:
                if self.signals.RELEASE_SIGNAL is None:
                    self.game_state = self.signals.PREVIOUS_STATE
                    self.signals.PREVIOUS_STATE = None
            elif self.game_state == GAME_STATE.DECISION:
                if self.flags.FINISH_GAME:
                    self.state = GENERAL_STATE.SEARCHING
                    self.game_state = GENERAL_STATE.INITIAL

    def controller(self,event):
        """
        Control Distributer based on current state
        """
        self.change_state()
        if(self.state == GENERAL_STATE.INITIAL):
            dprint("INITIAL")
            self.init_start()
        elif(self.state == GENERAL_STATE.SEARCHING):
            dprint("SEARCHING")
            self.search_for_target()
        elif (self.state == GENERAL_STATE.MOVING_TO_HUMAN):
            dprint("MOVING TO HUMAN")
            self.flags.set(human_reach=True)
        elif (self.state == GENERAL_STATE.ASKING_HUMAN):
            dprint("ASKING_HUMAN")
            self.prompt_Q()
        elif (self.state == GENERAL_STATE.PLAYING_GAME):
            dprint("PLAYING_GAME")
            if(self.game_state == GAME_STATE.QUESTION):
                dprint("GAME: QUESTION")
                self.prompt_game_Q()
            elif(self.game_state == GAME_STATE.ANSWER):
                dprint("GAME: ANSWER")
                self.verify_ans()
            elif(self.game_state == GAME_STATE.DECISION):
                dprint("GAME: DECISION")
                self.update_backend()

    def run(self):
        rospy.Timer(rospy.Duration(0.1), self.controller)
        rospy.spin()


gameboi = None
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    gameboi = GameBoi()
    gameboi.run()
