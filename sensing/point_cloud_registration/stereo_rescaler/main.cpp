#include <stereo_rescaler/stereo_rescaler.h>
#include <bnerf_utils/bnerf_utils.h>


int main(int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");
    bnerf::StereoRescaler rescaler(nh);
    
    ros::spin();
    return 0;
}