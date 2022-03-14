#!/home/turtlebot/anaconda3/envs/gui/bin/python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import sys
import signal
import numpy as np
from util import COLOR2TAG

import pupil_apriltags as apriltag
import cv2

from collections import defaultdict

"""
* Filename: game_checker.py
* Student: Harsh Deshpande, hdeshpande@ucsd.edu; Daniel Beaglehole, dbeaglehole@ucsd.edu; Divyam Bapna, dbapna@ucsd.edu; Chao Chi Cheng, cccheng@ucsd.edu
* Project #6:  GameBoi
*
* Description: This is the checker file which implenets the FSM used to check the sequnce of colours shown by the human.
*              It uses a FSM where current state is held till a new (different from last seen tag) is seen. If tag
*              according to the sequence, the state updates to finding the next colour, whereas if its wrong, the 
*              checker resets to checking the sequece from the beginning.
*
*
*How to use:
* Build:
*   catkin build
*   source ~/catkin_ws/devel/setup.bash
* Usage:
*   rosrun gameboi game_checker.py
*   Make sure you set a valid colour sequence in the 'main' function of this file before runnign above command.
* Requirement:
*   Make sure every python files permission is set properly
"""

def sigint_handler(signal, frame):
	sys.exit(0)

class GameCheckerFSM:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('GameCheckerFSM')

        # Subscribe to the /blobs topic
        # self.blobs_sub = rospy.Subscriber('/blobs', Blobs, self.blobs_cb)
        self.detector = apriltag.Detector(nthreads=2,quad_decimate=1,families='tag36h11')
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.april_cb)

        #Parameters for speed and control
        self.last_blob = None
        self.last_tag = None
        self.seq = None
        self.seq_len = 0
        self.started = False
        self.index = 0.0

        #Store Game Details here
        self.colour_to_tag = COLOR2TAG.COLORTAGS
        self.tag_to_colour = {}
        for color in self.colour_to_tag:
            self.tag_to_colour[self.colour_to_tag[color]] = color

        #Communicating with main
        self.main_sub = rospy.Subscriber('/game_check_state',String,self.change_state,queue_size=1)
        self.main_pub = rospy.Publisher('/game_checker',String,queue_size=1)



    def change_state(self,comm_string):
        data = comm_string.data
        # print(data)
        if data=="end":
            self.started = False
        else:
            words = data.split(" ")
            assert words[0]=="start"
            self.seq = []
            for i in words[1:]:
                self.seq.append(self.colour_to_tag[i])
            self.find_index = 0
            print(self.seq)
            self.main_pub.publish("LOOKING FOR C0LOURS NOW")
            self.started = True
        return

    def set_seq(self,seq):
        # assert 1==0 #Shouldnt be called
        self.seq = []
        for i in seq:
            self.seq.append(self.colour_to_tag[i])
        self.seq_len = len(seq)
        self.find_index = 0
        self.started = True
        return


    """ 
        
        The callback function for the /blobs topic.
        This is called whenever we receive a message from /blobs.
        It finds the center of each found RED (PINK but named Red) blob.
        
        It finds the mean of all blob center points to detect the goal of movement. If the
        maximum blob size is greater than threshold, it asserts that the bot has reached its destination.
    """
    def get_tag_size(self,corners):
        corners = np.array(corners)
        # print(corners)
        l1 = np.sqrt(np.sum((corners[0]-corners[1])**2))
        l2 = np.sqrt(np.sum((corners[0]-corners[2])**2))
        # print(l1*l2/10)
        return l1*l2/10

    def april_cb(self,image): #Callback to check for tags shown by humans
        if not self.started: return 
        #Detecting tags
        im = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        im_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        result = self.detector.detect(im_gray)
        detected_tag = None
        max_size = -1e6
        for tag in result:
            size_tag = self.get_tag_size(tag.corners)
            #Getting the tag with largest size (dominant tag assumed to be largest)
            if size_tag>max_size:
                detected_tag = tag.tag_id
                max_size = size_tag
        if detected_tag==None: return
        if not self.last_tag or detected_tag!=self.last_tag:
            self.last_tag = detected_tag
            if detected_tag>6 or detected_tag<1: 
                return
            #Correct Tag Found
            if detected_tag==self.seq[self.find_index]:
                send_string = "Found %s"%(self.tag_to_colour[detected_tag])
                self.find_index+=1
                #Game Completed
                if self.find_index==len(self.seq):
                    send_string += "\nGame Passed"
                    self.started = False
                self.main_pub.publish(send_string)
                print(send_string)
            #Wrong tag found
            else:
                self.main_pub.publish("Found %s,Wrong Sequence, please restart"%(self.tag_to_colour[detected_tag]))
                self.find_index=0 #Resetting
                self.last_tag = None
                print("Found %s,Wrong Sequence, please restart"%(detected_tag))


    def blobs_cb(self, blobsIn):
        #Deprecated function, not used in this project, kept for legacy
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
        Empty Function as all control decision moved to 'gameboi.py' file
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
    game_checker.set_seq(['Red','Blue','Green'])
    game_checker.run()