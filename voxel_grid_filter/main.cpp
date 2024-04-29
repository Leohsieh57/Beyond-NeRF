#include <ros/ros.h>
#include <glog/logging.h>
#include <voxel_grid_filter/voxel_grid_filter.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~"); 
    bnerf::VoxelGridFilter filter(nh);

    ros::spin();
}