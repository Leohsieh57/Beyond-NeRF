#include <glog/logging.h>
#include <dust3r/dust3r_tester.h>


int main (int argc, char **argv)
{
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~"); 

    bnerf::Dust3rTester test1(nh, "cam02");
    bnerf::Dust3rTester test2(nh, "cam03");

    ros::spin();
    return 0;
}