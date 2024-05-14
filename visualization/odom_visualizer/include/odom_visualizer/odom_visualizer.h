#ifndef __BNERF_ODOM_VISUALIZER_H__
#define __BNERF_ODOM_VISUALIZER_H__


#include <ros/ros.h>
#include <bnerf_utils/typedef.h>
#include <geometry_msgs/Transform.h>


namespace bnerf
{
    class OdomVisualizer
    {
        public:
        OdomVisualizer(ros::NodeHandle & );
        
        private:
        void OdomCallBack(const geometry_msgs::TransformStamped &);

        ros::Subscriber odom_sub_;
        ros::Publisher pose_pub_;

        mutex pose_mutex_;
        map<ros::Time, SE3d> poses_;
    };
}

#endif // __BNERF_GRAPH_VISUALIZER_H__
