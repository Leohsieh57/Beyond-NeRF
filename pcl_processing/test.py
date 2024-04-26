#!/usr/bin/env python3
import rospy

from pcl_processing.srv import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform


class RegistTester:
    def __init__(self, topic):
        self.last_scan = PointCloud2()
        rospy.wait_for_service('align_point_clouds')
        try:
            self.service = rospy.ServiceProxy('align_point_clouds', pcl_srv)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        self.sub = rospy.Subscriber(topic, PointCloud2, self.scan_callback)
        self.pub1 = rospy.Subscriber("target", PointCloud2, queue_size=10)
        self.pub2 = rospy.Subscriber("source", PointCloud2, queue_size=10)


    def scan_callback(self, msg):
        trans = Transform()
        trans.rotation.w = 1 # set to identity

        if len(self.last_scan.data) == 0:
            self.last_scan = msg
            return 
        
        res: pcl_srvResponse
        res = self.service(self.last_scan, msg, trans)

        print("transformation:")
        print(res.result_transform)
        
        self.last_scan = msg



if __name__ == '__main__':
    rospy.init_node('dust3r_test_node')
    tester = RegistTester("scan_filter/filtered_points")
    
    rospy.spin()