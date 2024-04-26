#!/usr/bin/env python3
import json
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from tqdm import tqdm
from bnerf_msgs.srv import LoadDUSt3R, LoadDUSt3RRequest, LoadDUSt3RResponse
from rospkg import RosPack
import glob
from datetime import datetime


class OfflineDUSt3RServer:
    def __init__(self):
        data_dir = rospy.get_param('dust3r/data_dir')
        self.data = {}
        for idx in ["02", "03"]:
            frame_id = "cam%s" % idx
            xyz_conf = os.path.join(data_dir, "image_" + idx, 'xyz_conf')
            npy_files = glob.glob(os.path.join(xyz_conf, "*.npy"))

            def line_to_nsecs(line:str):
                secs, nsecs = line.split('.')
                secs = datetime.strptime(secs, "%Y-%m-%d %H:%M:%S")
                return int(secs.timestamp() * 1e9) + int(nsecs)

            lines = os.path.join(data_dir, "image_" + idx, 'timestamps.txt')
            lines = open(lines, 'r').readlines()
            to_idx = {line_to_nsecs(line): i for i, line in enumerate(lines)}
            print(list(to_idx))

            #self.data["cam%s" % cam_idx] = "image_%s" % cam_idx

    

            
if __name__ == '__main__':
    rospy.init_node('dust3r_server_node')
    server = OfflineDUSt3RServer()