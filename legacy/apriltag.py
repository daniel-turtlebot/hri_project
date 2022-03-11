#!/usr/bin/env python

import rospy

# import apriltag
# from pupil_apriltags import Detector
import pupil_apriltags as apriltag
# import dt_apriltags as apriltag

import sys
sys.path.insert(0, '/home/turtlebot/anaconda3/envs/gui/lib/python3.9/site-packages/')

# run this on terminal before launch this file
# export PYTHONPATH=/home/turtlebot/anaconda3/envs/gui/lib/python3.9/site-packages/:$PYTHONPATH
import cv2


from sensor_msgs.msg import Image

import numpy as np
import cv2
import time



class aptag:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('apriltag')
        self.detector = apriltag.Detector(nthreads=1)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.april_cb)

        
        
        
    def april_cb(self,image):
        
        im = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        im_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        result = self.detector.detect(im_gray)



        print(result)


    def run(self):
        rospy.spin()



if __name__ == '__main__':
    aptag = aptag()
    aptag.run()

