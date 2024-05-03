#!/usr/bin/env python3
import rospy
from lib import run_kitti2bag


            
if __name__ == '__main__':
    rospy.init_node('kitti2bag_node')

    kwargs = rospy.get_param('~')
    run_kitti2bag(**kwargs)