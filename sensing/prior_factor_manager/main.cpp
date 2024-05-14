#include <ros/ros.h>
#include <glog/logging.h>
#include <prior_factor_manager/prior_factor_manager.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");
    bnerf::PriorFactorManager pfm(nh);

    ros::spin();
}