#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from bnerf_msgs.msg import GraphBinaryEdge
import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_transforms(n=10, path_length=10):
    times = np.linspace(0, path_length, n)
    transforms = []
    for i in range(1, n):
        transform = TransformStamped()
        transform.transform.translation.x = np.random.normal(1, 0.1)  # 1 meter apart with noise
        transform.transform.rotation.w = 1  # No rotation, identity quaternion
        transform.header.stamp = rospy.Time(times[i])
        transform.header.frame_id = f"pose_{i-1}"
        transform.child_frame_id = f"pose_{i}"
        transforms.append(transform)
    return transforms

def publisher():
    rospy.init_node('transform_publisher', anonymous=True)
    pub = rospy.Publisher('graph_binary_edge', GraphBinaryEdge, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    transforms = generate_transforms()

    while not rospy.is_shutdown():
        for i, trans_stamped in enumerate(transforms):
            msg = GraphBinaryEdge(transform=trans_stamped.transform) #trans from 2 to 1
            msg.header2 = trans_stamped.header
            msg.header2 = trans_stamped.header
            trans_stamped
            msg.header1.stamp = rospy.Time.now()  # Assuming t2 is the current time for simplicity
            pub.publish(msg)
            rospy.loginfo(f"Published transformation from {msg.header1.frame_id} to {transform.child_frame_id}")
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

