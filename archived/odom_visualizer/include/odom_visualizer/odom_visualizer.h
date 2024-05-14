#ifndef __BNERF_ODOM_VISUALIZER_H__
#define __BNERF_ODOM_VISUALIZER_H__

#include <ros/ros.h>
#include <bnerf_utils/typedef.h>
#include <geometry_msgs/TransformStamped.h>

namespace bnerf
{
    class OdomVisualizer
    {
    public:
        OdomVisualizer(ros::NodeHandle &);

    private:
        void OdomCallBack(const geometry_msgs::TransformStamped &);
        void PublishOdometry(const ros::TimerEvent &);
        vector<SE3d> GetOdometries();

        ros::Subscriber odom_sub_;
        ros::Publisher pose_pub_;
        ros::Publisher path_pub_;
        ros::Timer timer_;

        mutex odom_mutex_;
        string frame_id_;
        map<ros::Time, SE3d> odoms_;
    };
}

#endif // __BNERF_ODOM_VISUALIZER_H__
