#include <ros/ros.h>
#include <glog/logging.h>
#include <odom_visualizer/odom_visualizer.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~"); 
    bnerf::OdomVisualizer visualizer(nh);

    ros::spin();
}