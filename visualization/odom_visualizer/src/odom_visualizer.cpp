#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <odom_visualizer/odom_visualizer.h>
#include <glog/logging.h>
#include <bnerf_utils/bnerf_utils.h>
#include <pcl/common/transforms.h>

namespace bnerf
{
    OdomVisualizer::OdomVisualizer(ros::NodeHandle & nh)
        : pose_pub_(nh.advertise<geometry_msgs::PoseArray>("trajectory", 128))
    {
        odom_sub_ = nh.subscribe("input_odom", 16, &OdomVisualizer::OdomCallBack, this);
    }


    void OdomVisualizer::OdomCallBack(const geometry_msgs::TransformStamped &)
    {

    }
}