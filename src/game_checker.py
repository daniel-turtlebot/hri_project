#!/usr/bin/env python
# ****************************************
#  Filename: blob_foll.py
#  Student: Harsh Deshpande
#  Lab #4: Blob Detector and Obstacle Avoidance


# Description: 
# This is the main run file for Lab 4 which demonstrates the turtlebot
# searching for a particular coloured paper (Pink in this case) and heads towards it
# after finding it. If on the way it encounters an obstacle, it heads backwards,
# turns a little, goes ahead, and again starts searching for the paper and heads towards it

# How to use:

# Colors.txt file, can be found in /params folder:
# Colour thresholds was defined in AB values using colorgui tool of cmvision.
# Values were calibereted in a tent on an afternoon, recalibaration may be required


# Put Following Lines in .bashrc for easy usage. Source this .bashrc in open terminal_size
# rosparam set /cmvision/color_file (blob_cather_path)/params/color.txt
# rosparam set /cmvision/mean_shift_on False
# rosparam set /cmvision/spatial_radius_pix 0
# rosparam set /cmvision/color_radius_pix 0
# rosparam set /cmvision/debug_on False


# Usage:
# roscore
# roslaunch turtlebot_bringup minimal.launch
# roslaunch astra_launch astra_pro.launch
# rosrun cmvision colorgui image:=/camera/rgb/image_raw
# rosrun blob_catcher blob_foll.py (After sourcing current workspace)

# ****************************************/

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

class GameCheckerFSM:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('GameCheckerFSM')

        # Subscribe to the /blobs topic
        self.blobs_sub = rospy.Subscriber('/blobs', Blobs, self.blobs_cb)
        # self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.processBump)
        self.state = 0
        #Parameters for speed and control
        self.bump = False #Set to true if bump detected
        self.last_blob = None
        self.seq = None
        self.seq_len = 0
        self.started = False
        self.index = 0.0

        #Communicating with main
        self.main_sub = rospy.Subscriber('/game_check_state',String,self.change_state,queue_size=1)
        self.main_pub = rospy.Publisher('/game_checker',String,queue_size=1)

    def change_state(self,data):
        data = data.data
        print(data)
        if data=="end":
            self.started = False
        else:
            words = data.split(" ")
            assert words[0]=="start"
            self.seq = words[1:]
            self.find_index = 0
            self.main_pub.publish("LOOKING FOR C0LOURS NOW")
            self.started = True
        return

    def set_seq(self,seq):
        assert 1==0 #Shouldnt be called
        self.seq = seq
        self.seq_len = len(seq)
        self.find_index = 0
        return

    
    """ This callback function sets a flag to true if bump is detected.
        This flag is used to make decisions by the controller.
    """
    # def processBump(self,bevent):
    #     if bevent.state==1:
    #         print("Bump Detected")
    #         self.bump = True

    """ The callback function for the /blobs topic.
        This is called whenever we receive a message from /blobs.
        It finds the center of each found RED (PINK but named Red) blob.
        
        It finds the mean of all blob center points to detect the goal of movement. If the
        maximum blob size is greater than threshold, it asserts that the bot has reached its destination.
    """
    def blobs_cb(self, blobsIn):
        # if self.state==3: return
        if not self.started: return
        if len(blobsIn.blobs)==0: return

        # print("Callback ")

        blob_freq = defaultdict(float)
        for blob in blobsIn.blobs:
            if blob.name not in self.seq: continue
            blob_freq[blob.name]+=blob.area


        if len(blob_freq.keys())==0: return #No blobs detected
        max_blob = max(blob_freq.keys(), key=lambda a: blob_freq[a]) #Maximum area blob
        # print(max_blob)

        if not self.last_blob or max_blob!=self.last_blob:
            self.last_blob = max_blob
            # print(max_blob,blob_freq[max_blob])
            if max_blob==self.seq[self.find_index]:
                send_string = "Found %s"%(max_blob)
                self.find_index+=1
                if self.find_index==len(self.seq):
                    send_string += "\nGame Passed"
                    self.started = False
                self.main_pub.publish(send_string)
            else:
                self.main_pub.publish("Found %s,Wrong Sequence, please restart"%(max_blob))
                self.find_index=0 #Resetting
        return

                

    """ The callback function for the /camera/depth/points topic.
        This is called whenever we receive a message from this topic.
        Prints a message when there's an object below the threshold.
        
        Due to unreliability of detecting obstacles, we decided to skip this and replace
        it with bumper sensor measurements.
     """
    # def pointcloud_cb(self, cloud):
    #     # Note that point[0] is x, point[1] is y, point[2] is z
    #     num_points = 0
    #     avg = 0
    #     count = 0
    #     pc = ros_numpy.numpify(cloud)
    #     height = pc.shape[0]
    #     width = pc.shape[1]
    #     np_points = np.zeros((height * width, 3), dtype=np.float32)
    #     np_points[:, 0] = np.resize(pc['x'], height * width)
    #     np_points[:, 1] = np.resize(pc['y'], height * width)
    #     np_points[:, 2] = np.resize(pc['z'], height * width)

    #     num_points = np.sum(np_points[:,2]<0.4)
    #     # print(num_points)
    #     if num_points>500:
    #         print("Near Something")
    #         self.state = 3
        
    #     else:
    #         self.state= self.state


    """
    Controller for the bot logic. It has 4 states:
    State 0: Finding the paper. Bot keeps rotating till it finds target
    State 1: Bot heads towards paper. Keeps correcting direction using a P controller.
    State bump: A bump was encountered. Bot backtracks, goes in a new direction and switches to state 0.
    State 3: Destination reached!
    """
    def controller(self):
        #As of now just prints
        return
        

    def run(self):
        r_time_f=rospy.Rate(10) #This rate helps the bot run smoother
        while not rospy.is_shutdown():
            self.controller()
            r_time_f.sleep()

if __name__ == '__main__':
    print("Running on Python ",sys.version)
    signal.signal(signal.SIGINT, sigint_handler) #Used to stop the bot safely using Ctrl+C
    game_checker = GameCheckerFSM()
    # game_checker.set_seq(['Red','Green','Red'])
    game_checker.run()