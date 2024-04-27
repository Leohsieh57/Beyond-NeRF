#!/usr/bin/env python3
import rospy

from bnerf_msgs.srv import PredictDUSt3R
from sensor_msgs.msg import Image


class DUSt3RTester:
    def __init__(self, topic = "cam02"):
        self.last_imgs = Image()
        rospy.wait_for_service('dust3r/predict_dust3r')
        try:
            self.service = rospy.ServiceProxy('dust3r/predict_dust3r', PredictDUSt3R)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        self.sub = rospy.Subscriber(topic, Image, self.image_callback)

    def image_callback(self, msg):
        if len(self.last_imgs.data):
            self.service(self.last_imgs, msg, True)

        self.last_imgs = msg



if __name__ == '__main__':
    rospy.init_node('dust3r_test_node')

    tester1 = DUSt3RTester("cam02")
    tester2 = DUSt3RTester("cam03")
    
    rospy.spin()