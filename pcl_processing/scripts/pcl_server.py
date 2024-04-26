#!/usr/bin/python3

from __future__ import print_function
import rospy
import pcl
import pcl_ros
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from pcl_srv.srv import point_cloud1, point_cloud2, transform

from pcl_processing.srv import point_cloud1, point_cloud2, transform
import rospy

def align_point_clouds(req):
    cloud1 = pcl.PointCloud()
    cloud2 = pcl.PointCloud()
    
    pcl_ros.point_cloud2.from_msg(req.point_cloud1, cloud1)
    pcl_ros.point_cloud2.from_msg(req.point_cloud2, cloud2)

    #NDT Algorithm
    reg = cloud1.make_NormalDistributionsTransform()
    #ICP Algorithm
    # reg = cloud1.make_IterativeClosestPoint()

    initial_guess = pcl_ros.transform_from_msg(req.transform)
    reg.setInputTarget(cloud1)
    reg.setInputSource(cloud2)
    reg.align(cloud2, initial_guess)

    result_transform = pcl_ros.transform_to_msg(reg.getFinalTransformation())
    
    return result_transform

def pcl_server():
    rospy.init_node('pcl_server')
    s = rospy.Service('align_point_clouds', transform, align_point_clouds)
    print("Ready to calculate pcl")
    rospy.spin()

if __name__ == "__main__":
    pcl_server()
