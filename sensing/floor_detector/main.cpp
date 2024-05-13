#include <floor_detector/floor_detector.h>
#include <bnerf_utils/bnerf_utils.h>


int main(int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh("~"); 
    bnerf::FloorDetector detector(nh);
    
    ros::spin();
    return 0;
}