#!/usr/bin/env python3
from os.path import join, exists

import numpy as np
import rospy

from bnerf_msgs.srv import PredictDUSt3R, PredictDUSt3RRequest, PredictDUSt3RResponse
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Header

import glob
from datetime import datetime
import cv2


class DUSt3RServer:
    def __init__(self):
        data_dir = rospy.get_param('dust3r/data_dir')
        self.data = {}

        def line_to_stamp(line:str):
            secs, nsecs = line.split('.')
            secs = datetime.strptime(secs, "%Y-%m-%d %H:%M:%S").timestamp()
            return rospy.Time(secs=int(secs), nsecs=int(nsecs))
        
        for idx in ["02", "03"]:
            cam_dir = join(data_dir, "image_" + idx)
            imgs = join(cam_dir, 'input_imgs', '*.png')
            imgs = sorted(glob.glob(imgs))

            lines = join(cam_dir, 'timestamps.txt')
            lines = open(lines, 'r').readlines()
            assert len(lines) == len(imgs)
            to_idx = {line_to_stamp(line): i for i, line in enumerate(lines)}

            xyzc = join(cam_dir, 'xyz_conf')
            assert exists(xyzc)
            self.data["cam" + idx] = dict(imgs=imgs, xyzc=xyzc, idx=to_idx)


    @staticmethod
    def get_cloud_msg(header, img_file, xyz_file):
        if not exists(img_file) or not exists(xyz_file):
            return PointCloud2()
        
        img = cv2.imread(img_file).reshape((-1, 3))
        img = img.astype(np.float32) / 255 
        xyz = np.load(xyz_file).reshape((-1, 4))

        fields = enumerate('x y z intensity b g r'.split())
        fields = [PointField(c, 4*i, PointField.FLOAT32, 1) for i, c in fields]
        return create_cloud(header, fields, np.hstack((xyz, img)))


    def service_callback(self, req: PredictDUSt3RRequest):
        res = PredictDUSt3RResponse()
        frame_id = req.img1.header.frame_id

        data = self.data[frame_id]
        t1 = req.img1.header.stamp
        t2 = req.img2.header.stamp

        idx1 = data["idx"][t1]
        idx2 = data["idx"][t2]

        dt = (t2 - t1) * 0.5
        header = Header(frame_id=frame_id, stamp=t1+dt)
        
        def get_kwargs(i):
            img_file = data['imgs'][eval("idx%d"%i)]
            xyz_file = "%010d_%010d_view%s.npy" % (idx1, idx2, i)
            xyz_file = join(data['xyzc'], xyz_file)
            return dict(header=header, img_file=img_file, xyz_file=xyz_file)
        
        res.cloud1 = DUSt3RServer.get_cloud_msg(**get_kwargs(1))
        res.cloud2 = DUSt3RServer.get_cloud_msg(**get_kwargs(2))
        return res

            
if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    dust3r = DUSt3RServer()
    s = rospy.Service('dust3r_node/predict_dust3r', PredictDUSt3R, dust3r.service_callback)
    
    rospy.spin()