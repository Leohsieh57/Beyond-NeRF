#include <ros/ros.h>
#include <glog/logging.h>
#include <graph_edge_manager/graph_edge_manager.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");
    bnerf::GraphEdgeManager manager(nh);

    ros::spin();
}