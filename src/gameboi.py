#!/home/turtlebot/anaconda3/envs/gui/bin/python3

from pickle import NONE
from tracemalloc import start
import rospy
import sys

print(sys.version_info)

import sensor_msgs.point_cloud2 as pc2

from cmvision.msg import Blob, Blobs
from sensor_msgs.msg import PointCloud2, Image
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
import numpy as np
# import cv2
import time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import threading
import gui_gameboi as guigb
import game_checker as gc
import wx
from utill import *

class GameBoi:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('GameBoi')
        # # Publish commands to the robot
        # self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.game_check_st_sub = rospy.Publisher('/game_check_state', String, queue_size=1)
        self.game_check_sub = rospy.Subscriber('/game_checker', String , self.game_check , queue_size=1)
        # self.camera_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw/compressed', CompressedImage, self.emo_rec_cp_cb, queue_size = 1)
        self.state = GENERAL_STATE.INITIAL
        self.game_state = GAME_STATE.NOT_PLAYING

        self.gui = None
        self.panel = None
        self.t1 = threading.Thread(target=self.start_gui)
        self.t1.start()
        while self.panel is None:
            rospy.sleep(0.2)
        self.game_check_str = None
        self.flags = FLAGS()
        self.signals = SIGNALS()

    def start_gui(self):
        self.gui = guigb.gui()
        print("GUI START")
        self.panel = self.gui.frame.panel
        self.gui.start()


    def pub_checker_state(self,text):
        s = String()
        s.data = text
        self.game_check_st_sub.publish(s)

    def game_check(self, s):
        self.game_check_str = s.data

    def update_gui_text(self,text):
        wx.CallAfter(self.panel.update_text, text)

    def change_state(self):

        if self.signals.RELEASE_SIGNAL is not None:
            if self.game_state == GAME_STATE.NOT_PLAYING:
                self.signals.PREVIOUS_STATE = None
                self.signals.RELEASE_SIGNAL = None
            elif self.game_state != GAME_STATE.SIGNALS:
                self.signals.PREVIOUS_STATE = self.game_state
                self.game_state = GAME_STATE.SIGNALS

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
            if self.panel.get_enter_flag():
                self.flags.set(start=True)
            # self.stop()
        elif(self.state == GENERAL_STATE.SEARCHING):
            print("SEARCHING")
            self.flags.set(human_found=True)
            # self.search_for_target()
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
                self.flags.set(finish_game=True)

    def prompt_Q(self):
        self.update_gui_text("Do You Want To Play A Game With Me?")
        want_to_play = False
        for _ in range(10):
            if self.panel.get_enter_flag():
                want_to_play = True
                break
            rospy.sleep(0.5)

        if want_to_play:
            self.update_gui_text("Let's Play The Game!")
            self.flags.set(human_reponse=True)
            rospy.sleep(2)
        else:
            self.flags.set(human_reponse=False)


    def prompt_game_Q(self):
        self.update_gui_text("The Seq is Pink Yellow Pink")
        self.flags.set(finish_game_q=True)
        rospy.sleep(3)

    def verify_ans(self):
        self.update_gui_text("Please Display The Color Sequence In Order")
        rospy.sleep(1)
        self.pub_checker_state("start Pink Yellow Pink")
        while(not self.game_check_str):
            print("FIND STRING")
            rospy.sleep(0.1)
        ps = None
        while(self.game_check_str != 'end'):
            if ps != self.game_check_str:
                self.update_gui_text(self.game_check_str)
                ps = self.game_check_str
            rospy.sleep(1)

        self.game_check_str = None
        self.flags.set(finish_ans_verify=True)



    def pub_bot_vel(self,linear,angular,time,repeat=1):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        for _ in range(repeat):
            self.velocity_pub.publish(t)
            rospy.sleep(time)




    def stop(self):
        self.pub_bot_vel(0, 0, 0, repeat=1)

    def search_for_target(self):
        self.pub_bot_vel(0, -0.5, 0, repeat=1)

    def avoid_obstacle(self):

        self.has_obstacle = False

        # Move back for 0.5 sec
        self.pub_bot_vel(-0.25, 0, 0.5)

        # Turn for 2.5 seconds
        self.pub_bot_vel(0, 0.6, 0.5, repeat=7)

        # Move around obstacle
        self.pub_bot_vel(0.3, -0.8, 0.4, repeat=6)

        self.stop()

    def get_twist_from_coord(self,diff_angle,diff_size):
        '''
        @param diff_angle the difference between center of the color blob and the center of camera frame in pixel coords
        @param diff_size the difference between desired target size and current target size (height)
        @return Twist
        '''

        t = Twist()
        t.linear.x = 1 if diff_size >=0 else -1  # x is moving back and forth
        t.angular.z = -1 if diff_angle >=0 else 1 # z is turning left and right

        if (abs(diff_angle) > self.target_window_size):
            # turn to center the target in the frame
            t.linear.x = 0
            t.angular.z *= 0.4
        elif abs(diff_size) > self.threshold_height :
            # move toward target
            t.linear.x *= 0.25
            t.angular.z = 0
        else:
            t.angular.z = 0
            t.linear.x = 0

        return t

    def run(self):
        rospy.Timer(rospy.Duration(0.1), self.controller)
        rospy.spin()



if __name__ == '__main__':
    l4e = GameBoi()
    l4e.run()
