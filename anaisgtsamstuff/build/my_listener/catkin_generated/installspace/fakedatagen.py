#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from my_listener.msg import TimeTransform
import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_transforms(n=10, path_length=10):
    """Generate noisy sequential transformations along a line."""
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
    pub = rospy.Publisher('transformations', TimeTransform, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    transforms = generate_transforms()

    while not rospy.is_shutdown():
        for i, transform in enumerate(transforms):
            msg = TimeTransform()
            msg.geo_trans = transform.transform
            msg.t1 = transform.header.stamp  # Simulating the timestamp for t1
            msg.t2 = rospy.Time.now()  # Assuming t2 is the current time for simplicity
            pub.publish(msg)
            rospy.loginfo(f"Published transformation from {transform.header.frame_id} to {transform.child_frame_id}")
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

