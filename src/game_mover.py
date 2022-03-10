#!/usr/bin/env python
import rospy

import sensor_msgs.point_cloud2 as pc2
from kobuki_msgs.msg import BumperEvent #Used to detect Bumper Events
from cmvision.msg import Blob, Blobs #Blob Detection
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys
import signal
import numpy as np
# import ros_numpy

from collections import defaultdict

def sigint_handler(signal, frame):
	sys.exit(0)

class GameMover():
    def __init__(self):
        rospy.init_node('Mover')
        self.started = False
        self.orig_color = 'Yellow'
        self.seq = None

        #--------------------Define all publishers and subscribers here---------------
        self.main_sub = rospy.Subscriber('/game_mover_state',String,self.change_state,queue_size=1)
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.main_pub = rospy.Publisher('/game_mover',String,queue_size=1)

        #-----------------Store all speed related info here--------------------------
        #Store speeds in these variables only
        self.state = 'Searching'
        self.index = None
        self.search_vel = Twist()
        self.search_vel.angular.z = 0.4
        self.move_vel = Twist()
        self.move_vel.linear.x = 0.5
        self.mov_scale = 0.005
        self.stationary = Twist()

    def blobs_cb(self, blobsIn):
        (self.goal_x, self.goal_y) = (0,0)
        if len(blobsIn.blobs)==0:
            return
        blob_to_find = self.seq[self.index] if self.index<len(self.seq) else self.orig_color
        for blob in blobsIn.blobs:
            n_blobs = 0
            goal_x = 0
            max_blob = 0
            print(blob.name)
            if blob.name == blob_to_find:
                goal_x += blob.area*blob.x
                n_blobs += blob.area
                max_blob = max(max_blob,abs(blob.top-blob.bottom)) #Max blob Size
            if max_blob>350: #If max blob greater than 350
                self.state = "Reached"
                if self.index==len(self.seq): self.state = "Finished"
        self.goal_x = goal_x/n_blobs #Mean of all detected blobs

        #Mean should be around 320 for paper to be in centre
        #Till this is true bot will keep rotating
        if self.state!=1 and self.goal_x>310 and self.goal_x<330: 
            print("Centered Blob") #Blob detected and centered
            self.state = "Moving"

    def set_seq(self,seq):
        self.seq = seq
        self.index = 0
        self.started = True
        return

    def change_state(self,comm_str):
        data = comm_str.data
        if data=='end':
            self.started = False
        else:
            seq = data.split(' ')
            assert seq[0]=='start'
            self.seq = seq[1:]
            self.started = True
            self.index = 0
        return
    
    def controller(self):
        if not self.started: 
            print("Not yet started")
            return
        else:
            # print(self.state)
            if self.state=="Searching":
                self.velocity_pub.publish(self.search_vel)
            elif self.state=="Moving":
                self.move_vel.z = (self.goal_x-330)*self.mov_scale
                self.velocity_pub.publish(self.move_vel)
            elif self.state=="Reached":
                print("Reached colour %s"%(self.index))
                rospy.sleep(5)
                self.index +=1
                self.state = "Searching"
            elif self.state == "Finished":
                self.main_pub.publish("Finished displaying sequence")
                self.velocity_pub.publish(Twist())
    
    def run(self):
        r_time_f=rospy.Rate(10) #This rate helps the bot run smoother
        while not rospy.is_shutdown():
            self.controller()
            r_time_f.sleep()


if __name__ == '__main__':
    print("Running on Python ",sys.version)
    signal.signal(signal.SIGINT, sigint_handler) #Used to stop the bot safely using Ctrl+C
    game_checker = GameMover()
    print("here")
    game_checker.set_seq(['Pink'])
    game_checker.run()