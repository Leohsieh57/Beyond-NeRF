#ifndef __BNERF_DUST3R_RESCALER_H__
#define __BNERF_DUST3R_RESCALER_H__


#include <bnerf_utils/typedef.h>
#include <bnerf_msgs/FloorCoeffs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


namespace bnerf
{
    class DUSt3RScaleFinder
    {
        public: 
        DUSt3RScaleFinder(ros::NodeHandle &);
        void SyncCallBack(const CloudXYZ::ConstPtr &, const bnerf_msgs::FloorCoeffs::ConstPtr &);
        void FloorCallBack(const bnerf_msgs::FloorCoeffs &);

        //parameter & node handle stuffs
        private:
        message_filters::Subscriber<CloudXYZ> src_cloud_sub_;
        message_filters::Subscriber<bnerf_msgs::FloorCoeffs> src_floor_sub_;
        message_filters::TimeSynchronizer<CloudXYZ, bnerf_msgs::FloorCoeffs> sync_;

        ros::Publisher scan_pub_;
        ros::Subscriber tgt_floor_sub_;

        Mat44d trans_;
        ros::Duration win_span_;
        mutex win_mutex_;
        vector<bnerf_msgs::FloorCoeffs> tgt_win_;

        //visualization stuff
        private:
        unique_ptr<ros::Publisher> viz_pub_;
    };
}

#endif