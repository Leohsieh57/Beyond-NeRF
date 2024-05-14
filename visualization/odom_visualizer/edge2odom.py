#!/usr/bin/env python3
import os
import PIL.Image
import rospy
import torch
import torchvision.transforms as tvf
import numpy as np


from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField, PointCloud2, Image
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from bnerf_msgs.msg import GraphBinaryEdge
from cv_bridge import CvBridge
from bnerf_msgs.msg import DUSt3RInfo
import PIL


class EdgeToOdom:
    def __init__(self, frame_id, edge_topic, odom_topic):
        self.frame_id = frame_id
        self.odom_pub = rospy.Publisher(odom_topic, TransformStamped, queue_size=10)
        self.edge_sub = rospy.Subscriber(edge_topic, GraphBinaryEdge, self.edge_call_back, queue_size=10)


    def edge_call_back(self, e: GraphBinaryEdge):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.transform = e.mean
        self.odom_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('dust3r_stereo_node')

    kwargs = rospy.get_param('~')
    dust3r = EdgeToOdom(**kwargs)
    
    rospy.spin()