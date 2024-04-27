#!/usr/bin/env python3
from os.path import join, exists

import numpy as np
import rospy

from bnerf_msgs.srv import PredictDUSt3R, PredictDUSt3RRequest, PredictDUSt3RResponse
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import glob
from datetime import datetime
import cv2


class DUSt3RServer:
    def __init__(self):
        data_dir = rospy.get_param('dust3r/data_dir')
        self.data = {}

        def line_to_nsecs(line:str):
            secs, nsecs = line.split('.')
            secs = datetime.strptime(secs, "%Y-%m-%d %H:%M:%S")
            return int(secs.timestamp() * 1e9) + int(nsecs)
        
        for idx in ["02", "03"]:
            cam_dir = join(data_dir, "image_" + idx)
            imgs = join(cam_dir, 'input_imgs', '*.png')
            imgs = sorted(glob.glob(imgs))

            lines = join(cam_dir, 'timestamps.txt')
            lines = open(lines, 'r').readlines()
            assert len(lines) == len(imgs)
            to_idx = {line_to_nsecs(line): i for i, line in enumerate(lines)}

            frame_id = "cam" + idx
            xyz_conf = join(cam_dir, 'xyz_conf')
            assert exists(xyz_conf)
            self.data[frame_id] = dict(imgs=imgs, xyz_conf=xyz_conf, idx=to_idx)

        self.pubs = {}
        for frame_id in self.data:
            topic = "dust3r/" + frame_id
            pub = rospy.Publisher(topic, PointCloud2, queue_size=10)
            self.pubs[frame_id] = pub


    def publish_pointcloud(self, header, views, ids):
        frame_id = header.frame_id
        imgs = [self.data[frame_id]['imgs'][i] for i in ids]

        imgs = [cv2.imread(img).reshape((-1, 3)) for img in imgs]
        imgs = [img.astype(np.float32) / 255 for img in imgs]
        xyzs = [np.load(view).reshape((-1, 4)) for view in views]

        points = [np.hstack(x) for x in zip(xyzs, imgs)]
        fields = 'x y z intensity b g r'.split()
        fields = [PointField(c, 4 * i, PointField.FLOAT32, 1) for i, c in enumerate(fields)]
                  
        msg = pc2.create_cloud(header, fields, np.vstack(points))
        self.pubs[frame_id].publish(msg)



    def service_callback(self, req: PredictDUSt3RRequest):
        res = PredictDUSt3RResponse()

        frame_id = req.img1.header.frame_id
        assert frame_id == req.img2.header.frame_id

        data = self.data[frame_id]
        t1 = req.img1.header.stamp.to_nsec()
        t2 = req.img2.header.stamp.to_nsec()

        if t1 not in data["idx"] or t2 not in data["idx"]:
            res.status = PredictDUSt3RResponse.INDEX_ERROR
            return res
        
        idx1, idx2 = data["idx"][t1], data["idx"][t2]
        get_npy = lambda i: "%010d_%010d_view%d.npy" % (idx1, idx2, i)
        files = [join(data["xyz_conf"], get_npy(i)) for i in (1, 2)]

        if not all(exists(file) for file in files):
            res.status = PredictDUSt3RResponse.PAIRING_ERROR
            return res
        
        res.status = PredictDUSt3RResponse.SUCCESS
        if req.publish:
            header = Header()
            header.frame_id = frame_id
            t1 = req.img1.header.stamp
            t2 = req.img2.header.stamp
            dt = (t2 - t1) * 0.5
            header.stamp = t1 + dt
            self.publish_pointcloud(header, files, [idx1, idx2])

        return res

            
if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    dust3r = DUSt3RServer()
    s = rospy.Service('dust3r/predict_dust3r', PredictDUSt3R, dust3r.service_callback)
    
    rospy.spin()