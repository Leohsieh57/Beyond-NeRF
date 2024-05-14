#ifndef __BNERF_GRAPH_VISUALIZER_H__
#define __BNERF_GRAPH_VISUALIZER_H__


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <bnerf_utils/typedef.h>
#include <bnerf_msgs/GraphIterationStatus.h>
#include <tf2_ros/transform_listener.h>


namespace bnerf
{
    class GraphVisualizer
    {
        public:
        GraphVisualizer(ros::NodeHandle &);
        
        private:
        void GraphStatusCallBack(const bnerf_msgs::GraphIterationStatus &);
        void ScanCallBack(const sensor_msgs::PointCloud2::ConstPtr &);

        ros::Subscriber stat_sub_;
        ros::Subscriber scan_sub_;
        ros::Publisher scan_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher path_pub_;

        mutex scan_mutex_;
        map<ros::Time, sensor_msgs::PointCloud2::ConstPtr> scan_msgs_;
        SE3d velo_to_imu_;
    };
}

#endif // __BNERF_GRAPH_VISUALIZER_H__
