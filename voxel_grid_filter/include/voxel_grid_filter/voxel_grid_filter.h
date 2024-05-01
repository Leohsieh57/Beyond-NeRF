#ifndef __BNERF_VOXEL_GRID_FILTER_H__
#define __BNERF_VOXEL_GRID_FILTER_H__


#include <ros/ros.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>


namespace bnerf
{
    class VoxelGridFilter
    {
        public:
        VoxelGridFilter(ros::NodeHandle &);
        
        private:
        bool IsPointInvalid(const pcl::PointXYZI &);
        void ScanCallBack(const sensor_msgs::PointCloud2 &);

        //std::string topic_;
        bool use_approx_;
        float leaf_size_;
        float square_min_range_;
        float square_max_range_;

        ros::Subscriber scan_sub_;
        ros::Publisher scan_pub_, info_pub_;
    };
}

#endif // __BNERF_VOXEL_GRID_FILTER_H__
