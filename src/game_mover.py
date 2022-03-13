#!/home/turtlebot/anaconda3/envs/gui/bin/python3
import rospy
# export PYTHONPATH=/home/turtlebot/anaconda3/envs/gui/lib/python3.9/site-packages/:$PYTHONPATH
import sensor_msgs.point_cloud2 as pc2
from kobuki_msgs.msg import BumperEvent #Used to detect Bumper Events
from cmvision.msg import Blob, Blobs #Blob Detection
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image

import sys
import signal
import numpy as np

import pupil_apriltags as apriltag
import cv2
from util import COLOR2TAG
# import ros_numpy

from collections import defaultdict

def sigint_handler(signal, frame):
	sys.exit(0)

class GameMover():
    def __init__(self):
        rospy.init_node('Mover')
        self.started = False
        self.seq = None
        self.blob_to_find = None
        self.orig_tag = COLOR2TAG.COLORTAGS['Human']
        self.goal_x = None

        #--------------------Define all publishers and subscribers here---------------
        self.main_sub = rospy.Subscriber('/game_mover_state',String,self.change_state,queue_size=1)
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.main_pub = rospy.Publisher('/game_mover',String,queue_size=1)
        # self.blobs_sub = rospy.Subscriber('/blobs', Blobs, self.blobs_cb)
        # self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.processBump)
        self.detector = apriltag.Detector(nthreads=2,quad_decimate=1,families='tag36h11')
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.april_cb)

        #-----------------Store all speed related info here--------------------------
        #Store speeds in these variables only
        self.state = 'Searching'
        self.index = None
        self.search_vel = Twist()
        self.search_vel.angular.z = 0.4
        self.move_vel = Twist()
        self.move_vel.linear.x = 0.2
        self.mov_scale = -0.005
        self.stationary = Twist()
    
    def processBump(self,bevent):
        if bevent.state==1:
            print("Bump Detected")
            self.bump = True
            self.state = "Reached"
            if self.index==len(self.seq): self.state = "Finished"

    def get_tag_size(self,corners):
        corners = np.array(corners)
        # print(corners)
        l1 = np.sqrt(np.sum((corners[0]-corners[1])**2))
        l2 = np.sqrt(np.sum((corners[0]-corners[2])**2))
        # print(l1*l2/10)
        return l1*l2/10

    def april_cb(self,image):
        if not self.started: return  
        im = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        im_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        result = self.detector.detect(im_gray)
        curr_tag = self.seq[self.index] if self.index<len(self.seq) else self.orig_tag
        for tag in result:
            print("Finding curr_tag ",curr_tag)
            if tag.tag_id == curr_tag:
                self.goal_x = tag.center[0]
                if self.get_tag_size(tag.corners)>500 or tag.center[1]<100:
                    self.state = "Reached"
                    self.goal_x = None
                    print("Reached blob %s"%(curr_tag))
                    if self.index==len(self.seq):
                        print("Finished") 
                        self.state = "Finished" 
        
        if not self.goal_x: return
        if self.state!="Moving":# and self.goal_x>280 and self.goal_x<340: 
            print("Centered Blob") #Blob detected and centered
            self.state = "Moving"

    def blobs_cb(self, blobsIn):
        (self.goal_x, self.goal_y) = (0,0)
        if len(blobsIn.blobs)==0:
            return
        self.blob_to_find = self.seq[self.index] if self.index<len(self.seq) else self.orig_color
        # print("Now finding blob %s"%(self.blob_to_find))

        n_blobs = 0.1 #Non zero to avoid divide by zero
        goal_x = 0
        max_blob = 0

        for blob in blobsIn.blobs:
            if blob.name == self.blob_to_find:
                # print("Inside loop")
                goal_x += blob.area*blob.x
                n_blobs += blob.area
                max_blob = max(max_blob,abs(blob.top-blob.bottom)) #Max blob Size
            if max_blob>150 and self.state=="Moving": #If max blob greater than 350
                self.state = "Reached"
                print("Reached blob %s"%(self.blob_to_find))
                if self.index==len(self.seq):
                    print("Finished") 
                    self.state = "Finished" 

        self.goal_x = goal_x/n_blobs #Mean of all detected blobs
        if n_blobs<1: return
        # print(n_blobs)
        #Mean should be around 320 for paper to be in centre
        #Till this is true bot will keep rotating
        # if self.blob_to_find=="Yellow": print(self.goal_x)
        if self.state!="Moving" and self.goal_x>280 and self.goal_x<340: 
            print("Centered Blob") #Blob detected and centered
            self.state = "Moving"

    def set_seq(self,seq):
        self.seq = seq
        self.index = 0
        self.started = True
        return

    def change_state(self,comm_str):
        self.colour_to_tag = COLOR2TAG.COLORTAGS2
        data = comm_str.data
        if data=='end':
            self.started = False
        else:
            seq = data.split(' ')
            assert seq[0]=='start'
            self.seq = []
            for i in seq[1:]:
                self.seq.append(self.colour_to_tag[i])
            self.started = True
            self.index = 0
        return
    
    def controller(self):
        if not self.started: 
            # print("Not yet started")
            return
        else:
            # print(self.state)
            if self.state=="Searching":
                self.velocity_pub.publish(self.search_vel)
            elif self.state=="Moving":
                print("Moving")
                self.move_vel.angular.z = (self.goal_x-330)*self.mov_scale*0.5
                self.velocity_pub.publish(self.move_vel)
            elif self.state=="Reached":
                # print("Reached colour %s"%(self.index))
                rospy.sleep(2)
                self.index +=1
                self.state = "Searching"
                self.goal_x = None
                
            elif self.state == "Finished":
                self.main_pub.publish("end")
                self.velocity_pub.publish(Twist())
                self.started = False
    
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
    #Tag_ids are 8 and 9
    # game_checker.set_seq([1,2,3])
    game_checker.run()