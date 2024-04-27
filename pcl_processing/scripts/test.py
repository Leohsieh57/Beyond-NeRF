#!/usr/bin/env python3
import rospy

from bnerf_msgs.srv import Registration, RegistrationRequest, RegistrationResponse
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform, TransformStamped
from copy import deepcopy
import tf2_ros


class RegistTester:
    def __init__(self, topic):
        self.caster = tf2_ros.TransformBroadcaster()
        self.last_scan = PointCloud2()
        rospy.wait_for_service('align_point_clouds')
        try:
            self.service = rospy.ServiceProxy('align_point_clouds', Registration)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        self.sub = rospy.Subscriber(topic, PointCloud2, self.scan_callback)
        self.tgt_pub = rospy.Publisher("target", PointCloud2, queue_size=10)
        self.src_pub = rospy.Publisher("source", PointCloud2, queue_size=10)


    def scan_callback(self, msg: PointCloud2):
        tgt_scan = deepcopy(self.last_scan)
        src_scan = deepcopy(msg)
        self.last_scan = msg

        if len(tgt_scan.data) == 0:
            return 
        
        init_guess = Transform()
        init_guess.rotation.w = 1 # set to identity
        
        res: RegistrationResponse
        res = self.service(target=tgt_scan, source=src_scan, init_guess=init_guess)
        T = res.transform

        t = rospy.Time.now()
        tgt_scan.header.stamp = t
        src_scan.header.stamp = t
        src_scan.header.frame_id = "source"
        tgt_scan.header.frame_id = "target"

        msg = TransformStamped(header=tgt_scan.header, child_frame_id="source", transform=T)
        self.caster.sendTransform(msg)
        self.tgt_pub.publish(tgt_scan)
        self.src_pub.publish(src_scan)
        


if __name__ == '__main__':
    rospy.init_node('regist_test_node')
    tester = RegistTester("scan_filter/filtered_points")
    
    rospy.spin()