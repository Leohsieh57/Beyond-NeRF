#include <glog/logging.h>
#include <scan_filter/scan_filter.h>


int main(int argc, char **argv) {
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh(ros::this_node::getName()); 

    beyond_nerf::ScanFilter filter(nh);
    ros::Subscriber sub = nh.subscribe("raw_scan", 1000, 
        &beyond_nerf::ScanFilter::ScanCallback, &filter);

    ros::spin();
    return 0;
}