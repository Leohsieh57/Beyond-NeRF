#ifndef __BNERF_NAVSAT_BIAS_H__
#define __BNERF_NAVSAT_BIAS_H__


#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <bnerf_utils/bnerf_utils.h>
#include <bnerf_msgs/GraphUnaryEdge.h>


namespace bnerf
{
    class NavSatBias
    {
        public:
        NavSatBias(ros::NodeHandle & );
        
        private:
        void NavSatCallBack(const sensor_msgs::NavSatFix &);

        bool utm_bias_set_;
        Vec3d utm_bias_;
        mutex utm_bias_mutex_;
        Mat33d gps_cov_;

        ros::Subscriber gps_sub_;
        ros::Publisher edge_pub_;
    };
}

#endif // __BNERF_GRAPH_EDGE_MANAGER_H__
