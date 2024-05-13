#ifndef __BNERF_FLOOR_DETECTOR_H__
#define __BNERF_FLOOR_DETECTOR_H__


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <bnerf_utils/bnerf_utils.h>
#include "floor.h"


namespace bnerf
{
    class FloorDetector
    {
        public:
        FloorDetector(ros::NodeHandle &, const string & ns);
        void ScanCallBack(const CloudXYZ::ConstPtr &) const;
        
        private:
        int threads_;
        double dist_;
        double prob_;
        
        ros::NodeHandle nh_;
        ros::Publisher  info_pub_;
        ros::Subscriber scan_sub_;
        unique_ptr<ros::Publisher> scan_pub_;

        mutex win_mutex_;
        ros::Duration win_span_;
        vector<Floor> floor_win_;
    };
}

#endif // __BNERF_FLOOR_DETECTOR_H__
