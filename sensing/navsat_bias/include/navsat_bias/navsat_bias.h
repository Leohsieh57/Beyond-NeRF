#ifndef __BNERF_VOXEL_GRID_FILTER_H__
#define __BNERF_VOXEL_GRID_FILTER_H__


#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <bnerf_msgs/GraphUnaryEdge.h>
#include <bnerf_utils/bnerf_utils.h>


namespace bnerf
{
    class NavSatBias
    {
        public:
        NavSatBias(ros::NodeHandle & );
        
        private:
        void NavSatCallBack(const sensor_msgs::NavSatFix &);

        ros::Publisher edge_pub_;
        ros::Subscriber gps_sub_;
        
        unique_ptr<const Vec3d> utm_bias_;
        mutex utm_bias_mutex_;

        Mat33d cov_;
    };
}

#endif // __BNERF_VOXEL_GRID_FILTER_H__
