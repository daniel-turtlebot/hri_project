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


def sigint_handler(signal, frame):
    print("STOP PROGRAM")
    gameboi.backend.end()
    if gameboi:
        if gameboi.gui:
            gameboi.gui.stop()
    sys.exit(0)



class GameBoi:

    def __init__(self):
        rospy.init_node('GameBoi')
       
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.game_check_pub = rospy.Publisher('/game_check_state', String, queue_size=1)
        self.game_check_sub = rospy.Subscriber('/game_checker', String , self.game_check_cb , queue_size=1)

        self.game_mover_pub = rospy.Publisher('/game_mover_state',String , queue_size=1)
        self.game_mover_sub = rospy.Subscriber('/game_mover',String, self.game_mover_cb, queue_size=1)
        

        self.emotion_pub = rospy.Publisher('/emotion_state', String, queue_size=1)
        self.emotion_sub = rospy.Subscriber('/emotion',String, self.emotion_cb, queue_size=1)
       
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

        self.t1 = threading.Thread(target=self.start_gui)
        self.t1.start()
        while self.panel is None:
            rospy.sleep(0.2)

    def reset(self):
        self.game_check_str = None
        self.game_mover_str = None
        self.resp_incorrect = None
        self.resp_correct = None
        self.game_seq = None
        self.reward_survey = 0
        self.reward_emotion = None
        self.num_fails = 0

    def audio_out_helper(self,text):
        self.audio.say(text)
        self.audio.runAndWait()

    def audio_out(self,text,anyc=False):

        if type(text) == list:
            text = text[randint(0,len(text)-1)]

        if anyc:
            self.t2 = threading.Thread(target=self.audio_out_helper, args=(text,))
            self.t2.start()
        else:
            self.audio_out_helper(text)
        

        
    def start_gui(self):
        self.gui = guigb.gui()
        print("GUI START")
        self.panel = self.gui.frame.panel
        self.gui.start()

    def get_game_action(self):
        
        actions = ACTIONS.get_game_action(self.backend.get_action())
        self.game_mode, self.resp_correct, self.resp_incorrect = actions
        self.game_seq = self.generate_game_seq()

    def generate_game_seq(self):
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
            self.audio_out('You Are Correct YA')
            self.pub_bot_vel(0,1,0.2,repeat=10)
            self.pub_bot_vel(0,-1,0.2,repeat=10)
            pass
            
    def start_game(self):
        
        self.reset()
        self.pre_survey()
        
        self.pub_emotion("start")
        self.update_gui_text("Let's Play The Game!")
        self.audio_out(SPEACH_STRING.Greetings)
        self.get_game_action()
        rospy.sleep(2)

    def update_gui_text(self,text):
        wx.CallAfter(self.panel.update_text, text)

    def game_mover_cb(self,s):
        self.game_mover_str = s.data

    def game_check_cb(self, s):
        self.game_check_str = s.data

    def emotion_cb(self, s):
        self.reward_emotion = float(s.data)

    def pub_mover(self,text):
        s = String()
        s.data = text
        self.game_mover_pub.publish(s)

    def pub_checker(self,text):
        s = String()
        s.data = text
        self.game_check_pub.publish(s)

    def pub_emotion(self,text):
        s = String()
        s.data = text
        self.emotion_pub.publish(s)
    
            
    def init_start(self):
        if self.panel.get_enter_flag():
            self.flags.set(start=True)
        self.stop()

    def search_for_target(self):
        # self.pub_mover("start Human")
        # while(self.game_mover_str != 'end'):
        #     print("WAITING SEARCHING")
        #     rospy.sleep(0.5)
        self.flags.set(human_found=True)
        

    def prompt_Q(self):
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
        if self.game_mode == GAME.VISUAL:
            self.audio_out(SPEACH_STRING.Instructions1)
            self.update_gui_text(self.game_seq)
            rospy.sleep(3)
        else:
            self.audio_out(SPEACH_STRING.Instructions2)
            self.update_gui_text("Please Follow The Robot And Remember The Color")
            self.pub_mover(self.game_seq)
            while(self.game_mover_str != 'end'):
                print("WAITING Q")
                rospy.sleep(0.5)

        self.flags.set(finish_game_q=True)

    def verify_ans(self):
        self.update_gui_text("Please Display The Colors In Order")
        rospy.sleep(1)
        self.pub_checker(self.game_seq)
        while(not self.game_check_str):
            print("FIND STRING")
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
        rospy.sleep(1)

        self.game_check_str = None
        self.flags.set(finish_ans_verify=True)

    def pre_survey(self):
        wx.CallAfter(self.gui.frame.show_pre_form)

        while self.gui.frame.form_panel is None:
            rospy.sleep(0.1)
        
        while not self.gui.frame.form_panel.button_clicked:
            rospy.sleep(0.5)
        self.pre_reward = tuple([ float(text) for text in self.gui.frame.form_panel.saved_rating])
        wx.CallAfter(self.gui.frame.show_game)
        
        self.panel = self.gui.frame.panel

    def update_backend(self):
        #TODO

        wx.CallAfter(self.gui.frame.show_post_form)
        self.pub_emotion("end")

        while self.gui.frame.form_panel is None:
            rospy.sleep(0.1)
        while not self.gui.frame.form_panel.button_clicked:
            rospy.sleep(0.5)

        while(self.reward_emotion is None):
            print("Waiting Reward EMO")
            rospy.sleep(0.5)

        self.post_reward = float(self.gui.frame.form_panel.saved_rating)

        self.backend.update_reward(self.post_reward,self.reward_emotion,self.num_fails)
        self.flags.set(finish_game=True)
        wx.CallAfter(self.gui.frame.show_game)
        self.panel = self.gui.frame.panel

    def stop(self):
        self.pub_bot_vel(0, 0, 0, repeat=1)

    def pub_bot_vel(self,linear,angular,time,repeat=1):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        for _ in range(repeat):
            self.velocity_pub.publish(t)
            rospy.sleep(time)

    def change_state(self):

        # if self.signals.RELEASE_SIGNAL is not None:
        #     if self.game_state == GAME_STATE.NOT_PLAYING:
        #         self.signals.PREVIOUS_STATE = None
        #         self.signals.RELEASE_SIGNAL = None
        #     elif self.game_state != GAME_STATE.SIGNALS:
        #         self.signals.PREVIOUS_STATE = self.game_state
        #         self.game_state = GAME_STATE.SIGNALS

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
                    self.game_state = GAME_STATE.NOT_PLAYING

    def controller(self,event):
        self.change_state()
        if(self.state == GENERAL_STATE.INITIAL):
            print("INITIAL")
            self.init_start()
        elif(self.state == GENERAL_STATE.SEARCHING):
            print("SEARCHING")
            self.search_for_target()
        elif (self.state == GENERAL_STATE.MOVING_TO_HUMAN):
            print("MOVING TO HUMAN")
            self.flags.set(human_reach=True)
        elif (self.state == GENERAL_STATE.ASKING_HUMAN):
            print("ASKING_HUMAN")
            self.prompt_Q()
        elif (self.state == GENERAL_STATE.PLAYING_GAME):
            print("PLAYING_GAME")
            if(self.game_state == GAME_STATE.QUESTION):
                print("GAME: QUESTION")
                self.prompt_game_Q()
            elif(self.game_state == GAME_STATE.ANSWER):
                print("GAME: ANSWER")
                self.verify_ans()
            elif(self.game_state == GAME_STATE.DECISION):
                print("GAME: DECISION")
                self.update_backend()

    def run(self):
        rospy.Timer(rospy.Duration(0.1), self.controller)
        rospy.spin()


gameboi = None
if __name__ == '__main__':
    # signal.signal(signal.SIGINT, sigint_handler)
    gameboi = GameBoi()
    gameboi.run()
