#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

def callback(data):
    points = np.array(list(PointCloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
    
    marker = Marker()
    marker.header = data.header
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    for point in points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]
        marker.points.append(p)

    marker_array = MarkerArray()
    marker_array.markers.append(marker)

    publisher.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('pointcloud_visualizer', anonymous=True)
    rospy.Subscriber("/transformed_pointcloud", PointCloud2, callback)
    publisher = rospy.Publisher("/transformed_pointcloud_marker", MarkerArray, queue_size=10)
    rospy.spin()