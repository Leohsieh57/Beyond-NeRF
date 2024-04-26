#!/usr/bin/env python3
from os.path import join, exists

import numpy as np
import rospy

from bnerf_msgs.srv import LoadDUSt3R, LoadDUSt3RRequest, LoadDUSt3RResponse
from sensor_msgs.msg import Image

import glob
from datetime import datetime

class OfflineDUSt3RTester:
    def __init__(self, frame_id = "cam02"):
        self.last_imgs = Image()
        rospy.wait_for_service('dust3r/load_dust3r')
        try:
            self.service = rospy.ServiceProxy('dust3r/load_dust3r', LoadDUSt3R)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        self.sub = rospy.Subscriber(frame_id, Image, self.image_callback)

    def image_callback(self, msg):
        if len(self.last_imgs.data) == 0:
            self.last_imgs = msg
            return
        
        res = self.service(self.last_imgs, msg, True)
        print(res.status)



if __name__ == '__main__':
    rospy.init_node('dust3r_test_node')

    test1 = OfflineDUSt3RTester("cam02")
    test2 = OfflineDUSt3RTester("cam03")
    # s = rospy.Service('dust3r/load_dust3r', LoadDUSt3R, dust3r.service_callback)
    
    rospy.spin()