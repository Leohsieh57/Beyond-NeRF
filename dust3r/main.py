#!/usr/bin/env python3
from os.path import join, exists

import numpy as np
import rospy


from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Header

import glob
from datetime import datetime
import cv2

from dust3r.inference import inference
from dust3r.model import AsymmetricCroCo3DStereo

if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    
    rospy.spin()