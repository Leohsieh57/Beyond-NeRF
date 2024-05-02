#ifndef __CLOUD_PUBLISHER_H__
#define __CLOUD_PUBLISHER_H__


#include <bnerf_utils/ros/publisher.hpp>
#include <sensor_msgs/PointCloud2.h>


namespace bnerf {
    class CloudPublisher : public Publisher<sensor_msgs::PointCloud2> {
        public:  
        CloudPublisher(ros::NodeHandle &, const string &topic,
            const string &id, const int &queue=10);

        void SetInput(CloudXYZ::ConstPtr, const ros::Time &t = ros::Time());
        void TimerCallBack(const ros::TimerEvent & e = ros::TimerEvent());
        ros::Timer CreateTimer(const double &);
        typedef shared_ptr<CloudPublisher> Ptr;

        private: 
        ros::NodeHandle &nh_;
        uint subs_;
        string id_;
        mutex mutex_;
        ros::Time stamp_;
        CloudXYZ::ConstPtr cloud_;
    };
}

#endif