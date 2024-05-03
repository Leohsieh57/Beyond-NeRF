#!/usr/bin/env python3
import rospy
from kitti2bag import run_kitti2bag

            
if __name__ == '__main__':
    rospy.init_node('test_node')
    run_kitti2bag()