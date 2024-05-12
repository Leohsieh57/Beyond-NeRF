#!/usr/bin/env python3
import PIL.Image
import rospy
import torch
import torchvision.transforms as tvf
import numpy as np

from dust3r.inference import *
from dust3r.utils.image import _resize_pil_image
from dust3r.image_pairs import make_pairs
from dust3r.model import AsymmetricCroCo3DStereo
from dust3r.utils.device import collate_with_cat

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2, Image
from std_msgs.msg import Header

import tf2_ros
from cv_bridge import CvBridge
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
        self.pub1 = rospy.Publisher('~view1/dust3r_cloud', PointCloud2, queue_size=10)
        self.pub2 = rospy.Publisher('~view2/dust3r_cloud', PointCloud2, queue_size=10)
        # print(self.model)


    @torch.no_grad()
    def stereo_call_back(self, img1, img2):
        # print(img1.header.stamp, img2.header.stamp)

        batch = self.prepare_batch(img1, img2)
        cloud1, cloud2 = self.predict_point_clouds(batch)
        cloud1.header = img1.header
        cloud2.header = img2.header
        self.pub1.publish(cloud1)
        self.pub2.publish(cloud2)



    def predict_point_clouds(self, batch):
        res = loss_of_one_batch(batch, self.model, None, self.device)
        pred1, pred2 = res['pred1'], res['pred2']

        clouds = []
        for b in range(2):
            xyz = [pred1['pts3d'][b], pred2['pts3d_in_other_view'][b]]
            if b:
                xyz.reverse()

            xyz = torch.cat(xyz, dim=0).reshape(-1, 3)

            conf = [pred1['conf'][b], pred2['conf'][b]]
            if b:
                conf.reverse()

            conf = torch.cat(conf, dim=0).reshape(-1, 1)
            # xyzi = torch.cat([xyz, conf], dim=-1).reshape(-1, 4)
            clouds.append(dict(xyz=xyz, conf=conf))

            # fields = enumerate('x y z intensity'.split())
            # fields = [PointField(c, 4*i, PointField.FLOAT32, 1) for i, c in fields]

            # cloud = create_cloud(Header(), fields, xyzi.detach().cpu().numpy())
            # clouds.append(cloud)

        x1 = clouds[0]['xyz']
        x2 = geotrf(self.to_view1[:3,:3], clouds[1]['xyz'])
        x1x2 = torch.sum(x1 * x2).item()
        x1x1 = torch.pow(x1, 2).sum().item()
        x2x2 = torch.pow(x2, 2).sum().item()

        x1t = (torch.sum(x1, dim=0) * self.to_view1[:3,-1]).sum().item()
        x2t = (torch.sum(x2, dim=0) * self.to_view1[:3,-1]).sum().item()

        a = np.array([[x1x1, -x1x2], [-x1x2, x2x2]])
        b = np.array([x1t, x2t])
        s1, s2 = np.abs(np.linalg.solve(a, b))
        print(s1, s2)
        clouds[0]['xyz'] *= s1
        clouds[1]['xyz'] *= s2

        msgs = []
        for cloud in clouds:
            xyzi = torch.cat([cloud['xyz'], cloud['conf']], dim=-1)
            
            fields = enumerate('x y z intensity'.split())
            fields = [PointField(c, 4*i, PointField.FLOAT32, 1) for i, c in fields]

            msg = create_cloud(Header(), fields, xyzi.detach().cpu().numpy())
            msgs.append(msg)

        return msgs


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

        msg = buf.lookup_transform(frame_view1, frame_view2, rospy.Time(0), rospy.Duration(5))
        q = msg.transform.rotation
        t = msg.transform.translation

        T = np.eye(4)
        T[:3,:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3,-1] = np.array([t.x, t.y, t.z])
        self.to_view1 = torch.from_numpy(T).to(self.device)
        # self.to_view1 = torch.inverse(self.to_view1)


if __name__ == '__main__':
    rospy.init_node('dust3r_node')
    kwargs = rospy.get_param('~')
    print(kwargs)
    dust3r = PredictorDUSt3R(**kwargs)
    
    rospy.spin()