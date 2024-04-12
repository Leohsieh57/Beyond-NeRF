#include <scan_filter/scan_filter.h>

namespace beyond_nerf
{
    ScanFilter::ScanFilter(ros::NodeHandle & nh)
        : pub_(nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 10))
    {
        // nh.param<std::string>("default_param", default_param, "default_value");
        // filter_.setLeafSize();
    }


    void ScanFilter::ScanCallback(
        const sensor_msgs::PointCloud2::ConstPtr & msg)
    {

    }
}
