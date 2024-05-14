#include <ros/ros.h>
#include <glog/logging.h>
#include <graph_visualizer/graph_visualizer.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~"); 
    bnerf::GraphVisualizer visualizer(nh);

    ros::spin();
}