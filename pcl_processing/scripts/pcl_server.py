#!/usr/bin/python3

from __future__ import print_function
import rospy
import pcl
import pcl_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

from pcl_processing.srv import *
import rospy
import tf2_ros


class TransformFinder:
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)

    def lookup_transform(self, tgt_header: Header, src_header: Header):
        trans_wt = self.buf.lookup_transform('world', tgt_header.frame_id, tgt_header.stamp)
        trans_ws = self.buf.lookup_transform('world', src_header.frame_id, src_header.stamp)
        print(trans_wt, trans_ws)

    def align_point_clouds(self, req: pcl_srvRequest):
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
        self.lookup_transform(req.point_cloud1.header, req.point_cloud2.header)
        res = pcl_srvResponse()

        res.result_transform.rotation.w = 1 #set to identity
        return res

def pcl_server():
    rospy.init_node('pcl_server')
    finder = TransformFinder()
    s = rospy.Service('align_point_clouds', pcl_srv, finder.align_point_clouds)
    print("Ready to calculate pcl")
    rospy.spin()

if __name__ == "__main__":
    pcl_server()
