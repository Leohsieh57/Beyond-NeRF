#!/usr/bin/python3

import rospy
from bnerf_msgs.srv import IntegrateIMURequest, IntegrateIMUResponse, IntegrateIMU
from bnerf_msgs.msg import GraphBinaryEdge
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, PoseArray
import numpy as np

def request_empty_time_stamps():
    rospy.wait_for_service('/integrate_imu')  # Wait for the service to be available
    try:
        print("Sending request at:", rospy.Time.now())
        integrate_imu = rospy.ServiceProxy('/integrate_imu', IntegrateIMU)
        request = IntegrateIMURequest()
        # request.stamps = [rospy.Time.now()]  # Set the stamps field to a list containing the current time
        response = integrate_imu(request)
        print("Received response at:", rospy.Time.now())
        print(response.edges)
        rospy.loginfo("Received edges")

        #convert to homogeneous coords
        edges = response.edges
        accumulated_transform = np.eye(4)
        accumulated_transforms_array = []
        for edge in edges:
            transform = edge.mean
            quaternion = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
            rotation = Rotation.from_quat(quaternion)
            translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])

            homogeneous_rotation = np.eye(4)
            homogeneous_rotation[:3, :3] = rotation.as_matrix()

            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = homogeneous_rotation[:3, :3]
            transformation_matrix[:3, 3] = translation

            accumulated_transform = np.dot(accumulated_transform, transformation_matrix)
            accumulated_transforms_array.append(accumulated_transform)

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()

        #convert homogeneous coords to pose array
        for accumulated_transform in accumulated_transforms_array:
            pose_msg = Pose()

            translation = accumulated_transform[:3, 3]
            quaternion = Rotation.from_matrix(accumulated_transform[:3, :3]).as_quat()

            pose_msg.position.x = translation[0]
            pose_msg.position.y = translation[1]
            pose_msg.position.z = translation[2]

            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            pose_array_msg.poses.append(pose_msg)
        
        print(pose_array_msg)
        #publish pose array
        pose_array_publisher.publish(pose_array_msg)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('service_client_node')

    pose_array_publisher = rospy.Publisher('/pose_array_topic', PoseArray, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        request_empty_time_stamps()
        rate.sleep()

    rospy.spin()