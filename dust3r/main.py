#!/usr/bin/env python3
from os.path import join, exists

import numpy as np
import rospy

from bnerf_msgs.srv import PredictDUSt3R, PredictDUSt3RRequest, PredictDUSt3RResponse
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField
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

            xyz_conf = join(cam_dir, 'xyz_conf')
            assert exists(xyz_conf)
            self.data["cam" + idx] = dict(imgs=imgs, xyz_conf=xyz_conf, idx=to_idx)


    def service_callback(self, req: PredictDUSt3RRequest):
        res = PredictDUSt3RResponse()

        frame_id = req.img1.header.frame_id
        assert frame_id == req.img2.header.frame_id

        data = self.data[frame_id]
        t1 = req.img1.header.stamp
        t2 = req.img2.header.stamp

        if t1 not in data["idx"] or t2 not in data["idx"]:
            return res
        
        ids = [data["idx"][t1], data["idx"][t2]]
        get_npy = lambda i: "%010d_%010d_view%d.npy" % (*ids, i)
        outputs = [join(data["xyz_conf"], get_npy(i)) for i in (1, 2)]

        if not all(exists(file) for file in outputs):
            return res
        
        fields = 'x y z intensity b g r'.split()
        fields = [PointField(c, 4 * i, PointField.FLOAT32, 1) for i, c in enumerate(fields)]

        dt = (t2 - t1) * 0.5
        header = Header(frame_id=frame_id, stamp=t1+dt)

        clouds = []
        for idx, file in zip(ids, outputs):
            img = self.data[frame_id]['imgs'][idx]
            img = cv2.imread(img).reshape((-1, 3))
            img = img.astype(np.float32) / 255 

            xyz = np.load(file).reshape((-1, 4))
            points = np.hstack((xyz, img))
            print(points.shape)
            msg = create_cloud(header, fields, points)
            clouds.append(msg)
        
        res.cloud1, res.cloud2 = clouds
        return res

            
if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    dust3r = DUSt3RServer()
    s = rospy.Service('dust3r_node/predict_dust3r', PredictDUSt3R, dust3r.service_callback)
    
    rospy.spin()