#ifndef __BNERF_GRAPH_VISUALIZER_H__
#define __BNERF_GRAPH_VISUALIZER_H__


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <bnerf_utils/typedef.h>
#include <bnerf_utils/ros/publisher.hpp>
#include <bnerf_msgs/GraphIterationStatus.h>
#include <tf2_ros/transform_listener.h>


namespace bnerf
{
    class GraphVisualizer
    {
        public:
        GraphVisualizer(ros::NodeHandle &);
        
        private:
        void ScanCallBack(const sensor_msgs::PointCloud2 &);

        ros::Subscriber scan_sub_;
        Publisher<bnerf_msgs::GraphIterationStatus> pub_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener tf_lis_;
    };
}

#endif // __BNERF_GRAPH_VISUALIZER_H__
