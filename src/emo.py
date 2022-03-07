#!/usr/bin/env python

import rospy

import json
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
import numpy as np
import time
import requests
import base64



class Lab4Example:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('emo')
        # self.camera_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.emo_rec_cb)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.emo_rec_cp_cb)
        self.ccount = 0
        self.base_url = 'http://ec2-54-68-48-154.us-west-2.compute.amazonaws.com:5000/'
        self.session = requests.Session()
        self.headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
        

  
        
    def emo_rec_cb(self, image):
        if self.ccount == 0:
            self.t3 = time.time()
        delay = rospy.get_time() - (image.header.stamp.secs + image.header.stamp.nsecs/1000000000)
        
        if self.ccount == 1999:
            print(time.time() - self.t3)
        if self.ccount == 2000:
            return
        print(delay,self.ccount)
        # if delay > 0.5:
        #     return 

        self.ccount = self.ccount + 1

        # print(image.data)
        # r = requests.get('http://ec2-54-68-48-154.us-west-2.compute.amazonaws.com:5000/add')
        # im = np.frombuffer(image.data, dtype=np.uint8).tostring().decode()
        
        im = base64.b64encode(image.data).decode('utf-8')
        data = {'height': image.height, 'width': image.width, 'image': im}
        response = self.session.post(self.base_url + 'image', json=data)
        

        # t1 = time.time()
        # im = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        # print(im)
        # if self.ccount % 2 == 0:
        #     im_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        
        # demography = self.emo(im)
        # t2 = time.time()
     


        # print(t2 - t1)
        
    def emo_rec_cp_cb(self, image):
        if self.ccount == 0:
            self.t3 = time.time()
        delay = rospy.get_time() - (image.header.stamp.secs + image.header.stamp.nsecs/1000000000)
        
        if self.ccount == 1999:
            print(time.time() - self.t3)
        if self.ccount == 2000:
            return
        print(delay,self.ccount)
        if delay > 0.5:
            return 

        self.ccount = self.ccount + 1

        # print(image.data)
        # r = requests.get('http://ec2-54-68-48-154.us-west-2.compute.amazonaws.com:5000/add')
        # im = np.frombuffer(image.data, dtype=np.uint8).tostring().decode()
        
        # im = base64.b64encode(image.data).decode('utf-8')
        # data = {'height': image.height, 'width': image.width, 'image': im}
        # response = self.session.post(f'{self.base_url}image', json=data)

        # print(image.data)
        im = base64.b64encode(image.data).decode('utf-8')
        base64_bytes = im.encode('utf-8')
        message_bytes = base64.b64decode(base64_bytes)
        

        # print(im)
        data = {'image': im}
        response = self.session.post(self.base_url + 'image', data=json.dumps(data), headers=self.headers)
        # np_arr = np.fromstring(image.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imwrite('test.jpg', image_np)
        # print(image_np)
        
        

        # t1 = time.time()
        # im = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        # print(im)
        # if self.ccount % 2 == 0:
        #     im_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        
        # demography = self.emo(im)
        # t2 = time.time()
     


        # print(t2 - t1)
        

    def run(self):
        # rospy.Timer(rospy.Duration(0.2), self.controller)
        rospy.spin()



if __name__ == '__main__':
    l4e = Lab4Example()
    l4e.run()

