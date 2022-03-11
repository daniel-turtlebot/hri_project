#!/usr/bin/env python




from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

import numpy as np
import requests
import base64
import rospy
import time
import json





class Emotion:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('emo')
        
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.emo_rec_cp_cb)
        self.emotion_sub = rospy.Subscriber('/emotion_state', String, self.emo_st_cb, queue_size=1)
        self.emotion_pub = rospy.Publisher('/emotion',String , queue_size=1)
        self.score = '0'
        self.base_url = 'http://ec2-54-68-48-154.us-west-2.compute.amazonaws.com:5000/'
        self.session = requests.Session()
        self.headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
        self.started = False
    
    def emo_st_cb(self, s):
        text = s.data
        if text == 'start':
            self.started = True
        else:
            self.started = False
            self.score = '0'
            self.pub_emotion(self.score)
            self.session.get(self.base_url + 'reset')

        

    def pub_emotion(self,text):
        s = String()
        s.data = text
        self.emotion_pub.publish(s)
        
    def emo_rec_cp_cb(self, image):
        if rospy.get_time() - (image.header.stamp.secs + image.header.stamp.nsecs/1000000000) > 3:
            return 
        data = {'image': base64.b64encode(image.data).decode('utf-8')}
        self.score = self.session.post(self.base_url + 'image', data=json.dumps(data), headers=self.headers)

    def run(self):
        # rospy.Timer(rospy.Duration(0.2), self.controller)
        rospy.spin()



if __name__ == '__main__':
    emotion = Emotion()
    emotion.run()

