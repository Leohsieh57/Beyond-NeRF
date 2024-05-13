#ifndef __BNERF_STEREO_RESCALER_H__
#define __BNERF_STEREO_RESCALER_H__


#include <bnerf_utils/typedef.h>
#include <bnerf_msgs/FloorCoeffs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


namespace bnerf
{
    class StereoRescaler
    {
        public: 
        StereoRescaler(ros::NodeHandle &);
        void SyncedCallBack(const CloudXYZ::ConstPtr &, 
            const bnerf_msgs::FloorCoeffs::ConstPtr &);
        void CoeffsCallBack(const bnerf_msgs::FloorCoeffs::ConstPtr &);

        //parameter & node handle stuffs
        private:
        message_filters::Subscriber<CloudXYZ> src_cloud_sub_;
        message_filters::Subscriber<bnerf_msgs::FloorCoeffs> src_floor_sub_;
        message_filters::TimeSynchronizer<CloudXYZ, bnerf_msgs::FloorCoeffs> sync_sub_;

        ros::Publisher scan_pub_;
        ros::Subscriber tgt_floor_sub_;

        Mat44d to_stereo_;
        ros::Duration win_span_;
        mutex win_mutex_;
        vector<bnerf_msgs::FloorCoeffs::ConstPtr> floor_win_;
    };
}

#endif