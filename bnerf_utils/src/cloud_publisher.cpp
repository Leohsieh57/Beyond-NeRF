#include <bnerf_utils/ros/cloud_publisher.h>


namespace bnerf {
    CloudPublisher::CloudPublisher(ros::NodeHandle &nh, 
        const string &topic, const string &id, const int &queue)
        : Publisher<sensor_msgs::PointCloud2>(nh, topic, queue)
        , nh_(nh)
        , subs_(0)
        , id_(id) {}


    void CloudPublisher::SetInput(
        CloudXYZ::ConstPtr cloud, const ros::Time &stamp) 
    {
        lock_guard<mutex> lock(mutex_);
        stamp_ = stamp;
        cloud_ = cloud;
    }


    ros::Timer CloudPublisher::CreateTimer(const double &dur) {
        return nh_.createTimer(ros::Duration(dur), 
            &CloudPublisher::TimerCallBack, this);
    }


    void CloudPublisher::TimerCallBack(const ros::TimerEvent &) {
        ros::Time stamp;
        CloudXYZ::ConstPtr cloud;
        {
            lock_guard<mutex> lock(mutex_);
            stamp = stamp_;
            cloud.swap(cloud_);
        }
        
        if (cloud) {
            toROSMsg(*cloud, msg());
            header.frame_id = id_;
        }

        uint subs = getNumSubscribers();
        if (subs_ < subs || (subs && cloud)) {
            if (stamp.is_zero())
                stamp = ros::Time::now();

            header.stamp = move(stamp);
            publish();
        }
        subs_ = subs;
    }

}