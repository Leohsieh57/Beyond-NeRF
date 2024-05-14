#!/usr/bin/env python3
import os
import rospy
import numpy as np

from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
from bnerf_msgs.msg import GraphIterationStatus
import PIL
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped


class GraphStatusGenerator:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('~input_scan', PointCloud2, self.scan_call_back, queue_size=16)
        self.stat_pub = rospy.Publisher('~graph_status', GraphIterationStatus, queue_size=16)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
        self.msg = GraphIterationStatus()


    def scan_call_back(self, msg: PointCloud2):
        t = msg.header.stamp
        trans: TransformStamped
        trans = self.tf_buf.lookup_transform('velo_link', 'world', t, rospy.Duration(5))

        pose = Pose()
        pose.orientation = trans.transform.rotation
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        self.msg.graph_stamps.append(t)
        self.msg.graph_states.append(pose)

        self.stat_pub.publish(self.msg)
        while len(self.msg.graph_stamps) > 200:
            self.msg.graph_stamps.pop(0)
            self.msg.graph_states.pop(0)

        
if __name__ == '__main__':
    rospy.init_node('status_generator_node')

    gen = GraphStatusGenerator()
    rospy.spin()