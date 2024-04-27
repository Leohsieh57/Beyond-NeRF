#!/usr/bin/python3

from __future__ import print_function
import rospy
# import pcl
import pcl_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3

from bnerf_msgs.srv import Registration, RegistrationRequest, RegistrationResponse
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np


class Registrator:
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)


    def align_point_clouds(self, req: RegistrationRequest):
        '''
        service structure changed, 
        please check bnerf_msgs/srv/Registration.srv
        '''
        # cloud1 = pcl.PointCloud()
        # cloud2 = pcl.PointCloud()
        
        # pcl_ros.point_cloud2.from_msg(req.point_cloud1, cloud1)
        # pcl_ros.point_cloud2.from_msg(req.point_cloud2, cloud2)

        # #NDT Algorithm
        # reg = cloud1.make_NormalDistributionsTransform()
        # #ICP Algorithm
        # # reg = cloud1.make_IterativeClosestPoint()

        # initial_guess = pcl_ros.transform_from_msg(req.transform)
        # reg.setInputTarget(cloud1)
        # reg.setInputSource(cloud2)
        # reg.align(cloud2, initial_guess)

        # result_transform = pcl_ros.transform_to_msg(reg.getFinalTransformation())
        
        res = RegistrationResponse()
        T = self.lookup_gps_transform(req.target.header, req.source.header)
        res.transform = T
        return res
    

    @staticmethod
    def msg_to_numpy(msg):
        if isinstance(msg, TransformStamped):
            msg = msg.transform

        q = msg.rotation
        t = msg.translation

        T = np.eye(4)
        T[:3,:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3,-1] = np.array([t.x, t.y, t.z])
        return T
    
    @staticmethod
    def numpy_to_msg(T):
        x, y, z, w = R.from_matrix(T[:3,:3]).as_quat()

        msg = Transform()
        msg.rotation = Quaternion(x, y, z, w)
        msg.translation = Vector3(*T[:3,-1])
        return msg

    
    def lookup_gps_transform(self, tgt_header: Header, src_header: Header):
        frame_id = tgt_header.frame_id

        Twt = self.buf.lookup_transform('world', frame_id, tgt_header.stamp)
        Tws = self.buf.lookup_transform('world', frame_id, src_header.stamp)
        
        Twt = Registrator.msg_to_numpy(Twt)
        Tws = Registrator.msg_to_numpy(Tws)
        Tts = np.linalg.inv(Twt) @ Tws

        return Registrator.numpy_to_msg(Tts)
        


if __name__ == "__main__":
    rospy.init_node('pcl_server')
    regist = Registrator()
    s = rospy.Service('align_point_clouds', Registration, regist.align_point_clouds)
    print("Ready to calculate pcl")
    rospy.spin()
