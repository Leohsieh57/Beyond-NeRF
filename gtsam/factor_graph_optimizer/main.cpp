#include <ros/ros.h>
#include <glog/logging.h>
#include <factor_graph_optimizer/factor_graph_optimizer.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");
    bnerf::FactorGraphOptimizer fgo(nh);

    ros::spin();
}