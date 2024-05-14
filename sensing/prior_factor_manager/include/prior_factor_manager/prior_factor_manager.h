#ifndef __BNERF_PRIOR_FACTOR_MANAGER_H__
#define __BNERF_PRIOR_FACTOR_MANAGER_H__


#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <bnerf_utils/bnerf_utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>


namespace bnerf
{
    class PriorFactorManager
    {
        public:
        PriorFactorManager(ros::NodeHandle & );
        
        private:
        typedef sensor_msgs::Imu Imu;
        typedef sensor_msgs::NavSatFix Gps;

        void SyncCallBack(const Imu::ConstPtr &, const Gps::ConstPtr &);

        message_filters::Subscriber<Imu> imu_sub_;
        message_filters::Subscriber<Gps> gps_sub_;
        message_filters::TimeSynchronizer<Imu, Gps> sync_sub_;

        bool utm_bias_set_;
        Vec3d utm_bias_;
        mutex utm_bias_mutex_;
        Mat33d gps_cov_;
        string map_frame_;

        ros::Publisher factor_pub_;
        tf2_ros::TransformBroadcaster caster_;
    };
}

#endif // __BNERF_GRAPH_EDGE_MANAGER_H__
