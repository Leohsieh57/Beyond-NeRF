#!/usr/bin/env python3
from os.path import join, exists

import numpy as np
import rospy


from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Header

import torch
import cv2

from dust3r.inference import inference
from dust3r.model import AsymmetricCroCo3DStereo


class PointCloudPredictor:
    def __init__(self, pretrain, device):
        self.device = torch.device(device)
        self.model = AsymmetricCroCo3DStereo.from_pretrained(pretrain).to(device)
        self.model.eval()
        # print(self.model)


if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    kwargs = rospy.get_param('~')
    print(kwargs)
    dust3r = PointCloudPredictor(**kwargs)
    
    rospy.spin()