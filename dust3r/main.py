#!/usr/bin/env python3
import PIL.Image
import rospy
import torch
import torchvision.transforms as tvf
import numpy as np

from dust3r.utils.image import load_images
from dust3r.inference import *
from dust3r.utils.image import _resize_pil_image
from dust3r.image_pairs import make_pairs
from dust3r.model import AsymmetricCroCo3DStereo
from dust3r.utils.device import to_cpu, collate_with_cat

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2, Image
from geometry_msgs.msg import Transform, TransformStamped
from std_msgs.msg import Header
import tf2_ros
from cv_bridge import CvBridge
import cv2
import PIL

from scipy.spatial.transform import Rotation as R


class PredictorDUSt3R:
    def __init__(self, pretrain, device, width, frame_view1, frame_view2, slop):
        self.device = torch.device(device)
        self.model = AsymmetricCroCo3DStereo.from_pretrained(pretrain).to(self.device)
        self.model.eval()
        self.width = width

        self.look_up_transfrom(frame_view1, frame_view2)
        self.subs = [Subscriber('~view%d/image' % i, Image) for i in (1, 2)]
        self.sync = ApproximateTimeSynchronizer(self.subs, 1, slop=slop)
        self.sync.registerCallback(self.stereo_call_back)

        self.img_norm = tvf.Compose([tvf.ToTensor(), tvf.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])
        # print(self.model)


    @torch.no_grad()
    def stereo_call_back(self, img1, img2):
        # print(img1.header.stamp, img2.header.stamp)

        batch = self.prepare_batch(img1, img2)
        self.predict_point_clouds(batch)
        pass



    def predict_point_clouds(self, batch):
        res = loss_of_one_batch(batch, self.model, None, self.device)
        pred1, pred2 = res['pred1'], res['pred2']

        clouds = []
        for b in range(2):
            xyz = [pred1['pts3d'][b], pred2['pts3d_in_other_view'][b]]
            xyz = torch.cat(xyz, dim=0)

            conf = [pred1['conf'][b], pred2['conf'][b]]
            conf = torch.cat(conf, dim=0).unsqueeze(-1)
            xyzi = torch.cat([xyz, conf], dim=-1).reshape(-1, 4)
            print(xyzi.shape)
            
            


    def prepare_batch(self, img1, img2):
        bridge = CvBridge()

        views = []
        def img_to_view(img):
            img = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
            H, W, C = img.shape

            img = PIL.Image.fromarray(img) 
            img =_resize_pil_image(img, self.width)
            W, H = img.size

            cx, cy = W//2, H//2
            halfw, halfh = ((2*cx)//16)*8, ((2*cy)//16)*8
            img = img.crop((cx-halfw, cy-halfh, cx+halfw, cy+halfh))
            view = dict(img=self.img_norm(img)[None], 
                        true_shape=np.int32([img.size[::-1]]), 
                        idx=len(views), instance=str(len(views)))

            views.append(view)

        img_to_view(img1)
        img_to_view(img2)
        return collate_with_cat(make_pairs(views))


    def look_up_transfrom(self, frame_view1, frame_view2):
        buf = tf2_ros.Buffer()
        lis = tf2_ros.TransformListener(buf)

        msg: TransformStamped
        msg = buf.lookup_transform(frame_view1, frame_view2, rospy.Time(0), rospy.Duration(5))
        q = msg.transform.rotation
        t = msg.transform.translation

        T = np.eye(4)
        T[:3,:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3,-1] = np.array([t.x, t.y, t.z])
        self.to_view1 = torch.from_numpy(T).to(self.device)


if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    kwargs = rospy.get_param('~')
    print(kwargs)
    dust3r = PredictorDUSt3R(**kwargs)
    
    rospy.spin()