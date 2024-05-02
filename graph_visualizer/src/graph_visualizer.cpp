#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <graph_visualizer/graph_visualizer.h>
#include <glog/logging.h>
#include <bnerf_utils/bnerf_utils.h>


namespace bnerf
{
    GraphVisualizer::GraphVisualizer(ros::NodeHandle & nh)
        : scan_sub_()
        , pub_(nh, "graph_iteration_status", 1)
        , buffer_()
        , tf_lis_(buffer_)
    {
        scan_sub_ = nh.subscribe("scan_input", 1, &GraphVisualizer::ScanCallBack, this);
    }


    void GraphVisualizer::ScanCallBack(const sensor_msgs::PointCloud2 & msg)
    {
        const auto & t = pub_.graph_stamps.emplace_back(msg.header.stamp);
        const auto trans = buffer_.lookupTransform("world", msg.header.frame_id, t);
        convert(trans.transform, pub_.graph_states.emplace_back());

        if (pub_.graph_states.size() == 5)
        {
            pub_.publish();
            pub_.graph_states.clear();
            pub_.graph_stamps.clear();
        }
    }
}