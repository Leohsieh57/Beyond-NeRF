#include <ros/ros.h>
#include <glog/logging.h>
#include <navsat_bias/navsat_bias.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");
    bnerf::NavSatBias navsat_bias(nh);

    ros::spin();
}