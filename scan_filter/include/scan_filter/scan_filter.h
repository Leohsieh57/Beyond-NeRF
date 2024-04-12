#ifndef __BEYOND_NERF_SCAN_FILTER_H__
#define __BEYOND_NERF_SCAN_FILTER_H__


#include <ros/ros.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>


namespace beyond_nerf
{
    class ScanFilter
    {
        public:
        ScanFilter(ros::NodeHandle &);
        void ScanCallback(const sensor_msgs::PointCloud2::ConstPtr &);
        
        private:
        ros::Publisher pub_;
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> filter_;
    };
}


#endif  // __BEYOND_NERF_SCAN_FILTER_H__
