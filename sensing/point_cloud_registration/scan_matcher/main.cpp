#include <scan_matcher/scan_matcher.h>
#include <bnerf_utils/bnerf_utils.h>


int main(int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    bnerf::ScanMatcher scan_matcher;
    
    ros::spin();
    return 0;
}