#include <scan_filter/scan_filter.h>
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>


namespace beyond_nerf
{
    ScanFilter::ScanFilter(ros::NodeHandle & nh)
        : pub_(nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 1000))
    {
        float leaf;
        LOG_ASSERT(nh.getParam("leaf_size", leaf)) << std::endl 
            << "can't find \"" << nh.resolveName("leaf_size")
            << "\"\taborting.. " << std::endl;

        filter_.setLeafSize(leaf, leaf, leaf);
    }


    void ScanFilter::ScanCallback(
        const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr input;
        input.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *input);

        filter_.setInputCloud(input);
        pcl::PointCloud<pcl::PointXYZI> output;
        filter_.filter(output);

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(output, output_msg);
        pub_.publish(output_msg);
        std::cout << "input: " << input->size() 
            << "\toutput: " << output.size() << std::endl;
    }
}
