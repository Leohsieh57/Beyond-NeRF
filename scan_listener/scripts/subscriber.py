#!/usr/bin/python3

from bnerf_msgs.msg import GraphIterationStatus
from sensor_msgs.msg import PointCloud2
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from tf import transformations

pointcloud2s = []
lidar_stamps = []
poses = []
publisher = None

def callback(data):
    global lidar_stamps, poses
    iteration = data.iteration
    exec_time = data.exec_time.to_sec()

    ind = 0
    for stamp in data.graph_stamps:
        rospy.loginfo("Graph stamp: %s", stamp)
        for pc_msg, pc_stamp in pointcloud2s:
            if stamp == pc_stamp:
                lidar_stamps.append((stamp))
                poses.append((data.graph_states[ind]))
        ind +=1

    for state in data.graph_states:
        rospy.loginfo("Graph state: %s", state)

def apply_pose_to_pointcloud(pose, pointcloud):
    translation = [pose.position.x, pose.position.y, pose.position.z]
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    transform_matrix = transformations.quaternion_matrix(rotation)
    transform_matrix[:3, 3] = translation
    transformed_points = []

    for point in PointCloud2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        point = np.array([point[0], point[1], point[2], 1.0])
        transformed_point = np.dot(transform_matrix, point)
        transformed_points.append(transformed_point[:3])

    new_pointcloud = PointCloud2.create_cloud_xyz32(pointcloud.header, transformed_points)
    publish_pointclouds(new_pointcloud)
    pass

def callback2(msg):
    global pointcloud2s
    pointcloud2s.append((msg, msg.header.stamp))

def publish_pointclouds(pointcloud):
    global publisher
    if publisher is None:
        publisher = rospy.Publisher("/transformed_pointcloud", PointCloud2, queue_size=10)
    publisher.publish(pointcloud)

def transform_pointclouds():
    #skip every other index
    for i in range(0, len(poses), 2):
        for pc_msg, pc_stamp in pointcloud2s:
            if lidar_stamps[i] == pc_stamp:
                apply_pose_to_pointcloud(poses[i], pc_msg)


def listener():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("/voxel_grid_filter_node/filtered_points", GraphIterationStatus, callback)
    rospy.Subscriber('/graph_visualizer_node/graph_iteration_status', PointCloud2, callback2)

    if len(lidar_stamps) > 0:
        transform_pointclouds()

    rospy.spin()

if __name__ == '__main__':
    listener()