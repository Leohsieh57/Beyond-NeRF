#ifndef __BNERF_FLOOR_DETECTOR_H__
#define __BNERF_FLOOR_DETECTOR_H__


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <bnerf_utils/bnerf_utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


namespace bnerf
{
    class FloorDetector
    {
        public:
        FloorDetector(ros::NodeHandle & );
        void ScanCallBack(const CloudXYZ::ConstPtr &) const;
        
        private:
        int threads_;
        double dist_;
        double prob_;
        
        ros::Publisher info_pub_;
        ros::Publisher floor_pub_;
        ros::Subscriber scan_sub_;
        unique_ptr<ros::Publisher> scan_pub_;
    };
}

#endif // __BNERF_FLOOR_DETECTOR_H__
