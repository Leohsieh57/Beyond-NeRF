#!/usr/bin/env python3
import os
import PIL.Image
import rospy
import torch
import torchvision.transforms as tvf
import numpy as np

from dust3r.inference import loss_of_one_batch
from dust3r.utils.image import _resize_pil_image
from dust3r.utils.device import collate_with_cat
from dust3r.image_pairs import make_pairs
from dust3r.model import AsymmetricCroCo3DStereo

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from bnerf_msgs.msg import DUSt3RInfo
import PIL


class DUSt3RNet:
    def __init__(self, pretrain, device, width, slop, min_confidence):
        self.device = torch.device(device)
        self.model_name = os.path.basename(pretrain)
        self.model = AsymmetricCroCo3DStereo.from_pretrained(pretrain).to(self.device)
        self.model.eval()
        self.width = width

        self.subs = [Subscriber('~view%d/image' % i, Image) for i in (1, 2)]
        self.sync = ApproximateTimeSynchronizer(self.subs, 1, slop=slop)
        self.sync.registerCallback(self.stereo_call_back)

        self.img_norm = tvf.Compose([tvf.ToTensor(), tvf.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])
        self.min_conf = min_confidence
        self.scan_pub = rospy.Publisher('~dust3r_cloud', PointCloud2, queue_size=10)
        self.info_pub = rospy.Publisher('~dust3r_info', DUSt3RInfo, queue_size=10)

        fields = enumerate('x y z intensity'.split())
        self.fields = [PointField(c, 4*i, PointField.FLOAT32, 1) for i, c in fields]


    @torch.no_grad()
    def stereo_call_back(self, img1, img2):
        torch.cuda.reset_max_memory_allocated(self.device)

        t1 = rospy.Time.now()
        batch = self.prepare_batch(img1, img2)
        cloud = self.predict_point_cloud(batch)
        t2 = rospy.Time.now()

        dt = img2.header.stamp - img1.header.stamp
        cloud.header = img1.header
        cloud.header.stamp += dt * 0.5
        self.scan_pub.publish(cloud)

        msg = DUSt3RInfo()
        msg.header = cloud.header
        msg.exec_time = t2-t1
        msg.memory_usage = torch.cuda.max_memory_allocated(self.device)
        msg.memory_usage /= 1024**3
        msg.model_name = self.model_name
        self.info_pub.publish(msg)


    def predict_point_cloud(self, batch):
        res = loss_of_one_batch(batch, self.model, None, self.device)

        pred1, pred2 = res['pred1'], res['pred2']
        xyz = [pred1['pts3d'], pred2['pts3d_in_other_view']]
        xyz = [x.detach().cpu() for x in xyz]
        xyz = torch.vstack(xyz)

        conf = [pred1['conf'], pred2['conf']]
        conf = [x.detach().cpu() for x in conf]
        conf = torch.vstack(conf)
        valid_ids = conf > self.min_conf

        conf = conf.unsqueeze(-1)[valid_ids]
        xyz = xyz[valid_ids]

        xyzi = torch.cat([xyz, conf], axis=-1)
        cloud = create_cloud(Header(), self.fields, xyzi.reshape(-1, 4).numpy())
        return cloud


    def prepare_batch(self, img1, img2):
        bridge = CvBridge()

        views = []
        def img_to_view(img):
            img = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

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

        return collate_with_cat(make_pairs(views, symmetrize=False))


if __name__ == '__main__':
    rospy.init_node('dust3r_stereo_node')

    kwargs = rospy.get_param('~')
    dust3r = DUSt3RNet(**kwargs)
    
    rospy.spin()