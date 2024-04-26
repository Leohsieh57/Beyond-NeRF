#!/usr/bin/python3

import rospy
import pcl
import pcl_ros
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger, TriggerResponse
from pcl_processing.srv import *

#using the ndt method
def point_cloud_callback(msg):

    ndt = pcl.registration.NormalDistributionsTransform()

    ndt.setTransformationEpsilon(0.01)
    ndt.setStepSize(0.1)
    ndt.setResolution(1.0)

    # convert msg to PCL point cloud
    pcl_pc = pcl.PointCloud()
    pcl_pc.from_msg(msg)

    # align
    aligned_cloud = ndt.align(pcl_pc)
    aligned_msg = aligned_cloud.to_msg()

    pub.publish(aligned_msg)
    pass

if __name__ == "__main__":
    rospy.init_node("point_cloud_alignment_node")

    # subscribe to input topic
    rospy.Subscriber("/input_point_cloud", PointCloud2, point_cloud_callback)

    # publish
    pub = rospy.Publisher("/aligned_point_cloud", PointCloud2, queue_size=10)

    rospy.spin()
